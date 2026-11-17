#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蚕房录像 · ESP32-P4  —— 图形界面版

双击「蚕视频录制器.bat」或本文件即可，不需要命令行。
默认把视频存到 D:\\蚕视频。

功能：
  · 实时预览（确认镜头对准蚕台）
  · 一键开始/停止，自动生成可播放的 AVI
  · 可选同时导出逐帧 jpg，方便做 YOLO 数据集
  · 自动给 P4 对时，解决文件名 1970 的问题
  · 断线自动重连
"""

import datetime
import io
import os
import sys
import threading
import time

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# 复用命令行版的封装/抓流代码（同目录）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p4_recorder import StreamReader, AviWriter, sync_time, human   # noqa: E402

try:
    from PIL import Image, ImageTk
    HAVE_PIL = True
except Exception:
    HAVE_PIL = False

DEFAULT_HOST = "192.168.1.10"
DEFAULT_DIR = r"D:\蚕视频"
PREVIEW_W, PREVIEW_H = 640, 360

# 长时间录像必须分段：一个 60GB 的单文件一旦损坏就全丢，标注和回放也不方便
SEG_MAX_BYTES = 4 * 1024 ** 3       # 4GB 一段
SEG_MAX_SECS = 30 * 60              # 或 30 分钟一段，先到先分


# ------------------------------------------------------------------ 录制线程
class Recorder(threading.Thread):
    def __init__(self, host, port, out_dir, base_name, want_frames, every):
        super().__init__(daemon=True)
        self.host, self.port = host, port
        self.out_dir = out_dir
        self.base = base_name
        self.want_frames = want_frames
        self.every = max(1, every)

        self.stop_flag = threading.Event()
        self.avi_path = os.path.join(out_dir, base_name + ".avi")   # 兼容旧引用
        self.cur_path = self.avi_path
        self.avi = None
        self.seg_index = 0
        self.seg_bytes = 0
        self.seg_t0 = time.time()
        self.jpg_dir = os.path.join(out_dir, base_name + "_frames") if want_frames else None

        self.frames = 0
        self.bytes = 0
        self.gaps = 0
        self.worst_gap = 0.0
        self.error = None
        self.state = "连接中"
        self.latest_jpg = None        # 给界面预览用，永远是最新一帧
        self.t_start = None

    # ---- 帧率等实时数据
    def stats(self):
        el = max((time.time() - self.t_start), 0.001) if self.t_start else 0.001
        return {
            "state": self.state,
            "elapsed": el,
            "frames": self.frames,
            "fps": self.frames / el,
            "bytes": self.bytes,
            "gaps": self.gaps,
            "error": self.error,
            "avr_kb": (self.bytes / max(self.frames, 1) / 1024.0),
        }

    # ---- 分段
    def _seg_path(self, idx):
        if idx == 0:
            return os.path.join(self.out_dir, self.base + ".avi")
        return os.path.join(self.out_dir, "%s_%03d.avi" % (self.base, idx))

    def _open_segment(self):
        self.cur_path = self._seg_path(self.seg_index)
        self.avi = AviWriter(self.cur_path, 1920, 1080)
        self.seg_bytes = 0
        self.seg_t0 = time.time()

    def _rotate(self):
        try:
            self.avi.close()
        except Exception:
            pass
        self.seg_index += 1
        self._open_segment()

    def run(self):
        os.makedirs(self.out_dir, exist_ok=True)
        if self.jpg_dir:
            os.makedirs(self.jpg_dir, exist_ok=True)

        try:
            self.state = "连接中"
            reader = StreamReader(self.host, self.port)
            self.state = "录制中"
            self.t_start = time.time()

            self._open_segment()
            last_t = time.time()

            while not self.stop_flag.is_set():
                try:
                    jpg = reader.next_frame()
                except Exception:
                    if self.stop_flag.is_set():
                        break
                    self.state = "重连中"
                    try:
                        reader.close()
                    except Exception:
                        pass
                    time.sleep(1.0)
                    if self.stop_flag.is_set():
                        break
                    reader = StreamReader(self.host, self.port)
                    self.state = "录制中"
                    last_t = time.time()
                    continue

                now = time.time()
                dt = now - last_t
                last_t = now
                if dt > 0.35:
                    self.gaps += 1
                    if dt > self.worst_gap:
                        self.worst_gap = dt

                self.frames += 1
                self.bytes += len(jpg)
                self.avi.add(jpg)
                self.seg_bytes += len(jpg)
                self.latest_jpg = jpg

                if self.avi.frames % 10 == 0:      # 每约 1 秒回填头部，崩溃最多丢 1 秒
                    self.avi.sync()
                if self.seg_bytes >= SEG_MAX_BYTES or (now - self.seg_t0) >= SEG_MAX_SECS:
                    self._rotate()

                if self.jpg_dir and (self.frames % self.every == 0):
                    try:
                        with open(os.path.join(self.jpg_dir, "frame_%06d.jpg" % self.frames), "wb") as f:
                            f.write(jpg)
                    except Exception:
                        pass

        except Exception as e:
            self.error = str(e)
            self.state = "出错"
        finally:
            try:
                if self.avi is not None and self.frames:
                    self.avi.close()
            except Exception as e:
                self.error = self.error or ("保存失败: %s" % e)
            if self.state != "出错":
                self.state = "已停止"


# ------------------------------------------------------------------ 界面
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("蚕房录像 · ESP32-P4")
        self.geometry("800x720")
        self.minsize(760, 680)
        self.configure(bg="#15181d")

        self.rec = None
        self._img_ref = None
        self._stop_ts = 0.0

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(200, self._tick)

    # ---------------- 布局
    def _build_ui(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TFrame", background="#15181d")
        style.configure("TLabel", background="#15181d", foreground="#c9d1d9",
                        font=("Microsoft YaHei UI", 10))
        style.configure("Title.TLabel", foreground="#e6edf3",
                        font=("Microsoft YaHei UI", 14, "bold"))
        style.configure("Hint.TLabel", foreground="#7d8590",
                        font=("Microsoft YaHei UI", 9))
        style.configure("Big.TButton", font=("Microsoft YaHei UI", 11, "bold"), padding=8)
        style.configure("TEntry", fieldbackground="#0d1117", foreground="#e6edf3")
        style.configure("TCheckbutton", background="#15181d", foreground="#c9d1d9")

        root = ttk.Frame(self, padding=14)
        root.pack(fill="both", expand=True)

        # ---- 状态栏先占位放在底部：pack 自下而上预留空间，永远不会被上面挤掉 ----
        self.detail2 = tk.Label(root, text="", anchor="w", bg="#0d1117", fg="#6e7681",
                                font=("Consolas", 9), padx=10, pady=6)
        self.detail2.pack(side="bottom", fill="x")
        self.detail = tk.Label(root, text="", anchor="w", bg="#0d1117", fg="#8b949e",
                               font=("Consolas", 9), padx=10)
        self.detail.pack(side="bottom", fill="x")
        self.status = tk.Label(root, text="就绪", anchor="w", bg="#0d1117", fg="#58a6ff",
                               font=("Consolas", 10), padx=10, pady=8)
        self.status.pack(side="bottom", fill="x")
        ttk.Frame(root, height=10).pack(side="bottom", fill="x")

        ttk.Label(root, text="蚕房录像器", style="Title.TLabel").pack(anchor="w")
        ttk.Label(root, text="ESP32-P4 网线抓流 · 零转码保存", style="Hint.TLabel").pack(anchor="w", pady=(0, 10))

        # 预览：外框用像素固定尺寸，避免 Label 在没有图片时按"字符"撑爆布局
        box = tk.Frame(root, bg="#0d1117", width=PREVIEW_W + 2, height=PREVIEW_H + 2,
                       highlightthickness=1, highlightbackground="#30363d")
        box.pack(fill="x")
        box.pack_propagate(False)
        self.preview = tk.Label(box, text="（等待画面…）", bg="#0d1117", fg="#484f58")
        self.preview.pack(expand=True)

        # 设置
        opt = ttk.Frame(root)
        opt.pack(fill="x", pady=(12, 0))
        opt.columnconfigure(1, weight=1)

        ttk.Label(opt, text="板子地址").grid(row=0, column=0, sticky="w", pady=3)
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(opt, textvariable=self.host_var, width=20).grid(row=0, column=1, sticky="w", padx=8)

        self.sync_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(opt, text="开始前给板子对时（修正文件名日期）",
                        variable=self.sync_var).grid(row=0, column=2, sticky="w")

        ttk.Label(opt, text="保存到").grid(row=1, column=0, sticky="w", pady=3)
        self.dir_var = tk.StringVar(value=DEFAULT_DIR)
        ttk.Entry(opt, textvariable=self.dir_var).grid(row=1, column=1, sticky="ew", padx=8)
        ttk.Button(opt, text="浏览…", command=self._pick_dir).grid(row=1, column=2, sticky="w")

        self.frames_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(opt, text="同时导出逐帧图片（做数据集用）",
                        variable=self.frames_var).grid(row=2, column=0, columnspan=2, sticky="w", pady=3)
        sub = ttk.Frame(opt)
        sub.grid(row=2, column=2, sticky="w")
        ttk.Label(sub, text="每").pack(side="left")
        self.every_var = tk.StringVar(value="10")
        ttk.Entry(sub, textvariable=self.every_var, width=5).pack(side="left", padx=3)
        ttk.Label(sub, text="帧存一张").pack(side="left")

        # 按钮
        btns = ttk.Frame(root)
        btns.pack(fill="x", pady=14)
        self.btn_start = ttk.Button(btns, text="▶  开始录制", style="Big.TButton", command=self._start)
        self.btn_start.pack(side="left", expand=True, fill="x", padx=(0, 6))
        self.btn_stop = ttk.Button(btns, text="■  停止", style="Big.TButton",
                                   command=self._stop, state="disabled")
        self.btn_stop.pack(side="left", expand=True, fill="x", padx=(6, 0))

        btns2 = ttk.Frame(root)
        btns2.pack(fill="x")
        ttk.Button(btns2, text="打开保存文件夹", command=self._open_dir).pack(side="left")
        ttk.Button(btns2, text="在浏览器里看直播", command=self._open_browser).pack(side="left", padx=8)

    # ---------------- 交互
    def _pick_dir(self):
        d = filedialog.askdirectory(initialdir=self.dir_var.get() or DEFAULT_DIR,
                                    title="选择保存文件夹")
        if d:
            self.dir_var.set(os.path.normpath(d))

    def _open_dir(self):
        d = self.dir_var.get()
        os.makedirs(d, exist_ok=True)
        try:
            os.startfile(d)
        except Exception as e:
            messagebox.showerror("打不开", str(e))

    def _open_browser(self):
        import webbrowser
        webbrowser.open("http://%s/" % self.host_var.get().strip())

    def _start(self):
        host = self.host_var.get().strip() or DEFAULT_HOST
        out = self.dir_var.get().strip() or DEFAULT_DIR
        try:
            every = max(1, int(self.every_var.get()))
        except ValueError:
            every = 10

        os.makedirs(out, exist_ok=True)
        if self.sync_var.get():
            sync_time(host, 80)

        base = "p4_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.rec = Recorder(host, 80, out, base, self.frames_var.get(), every)
        self.rec.start()

        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="normal")

    def _stop(self):
        if self.rec and self.rec.is_alive():
            self.rec.stop_flag.set()
            self.status.configure(text="正在保存并关闭文件…", fg="#d29922")
        self.btn_stop.configure(state="disabled")

    def _on_close(self):
        if self.rec and self.rec.is_alive():
            if not messagebox.askokcancel("正在录制", "还在录制，确定要停止并退出吗？"):
                return
            self.rec.stop_flag.set()
            self.rec.join(timeout=5)
        self.destroy()

    # ---------------- 刷新
    def _tick(self):
        rec = self.rec
        if rec is not None:
            s = rec.stats()

            # 预览
            if HAVE_PIL and rec.latest_jpg is not None:
                try:
                    im = Image.open(io.BytesIO(rec.latest_jpg))
                    im.thumbnail((PREVIEW_W, PREVIEW_H))
                    self._img_ref = ImageTk.PhotoImage(im)
                    self.preview.configure(image=self._img_ref, text="", width=im.width, height=im.height)
                except Exception:
                    pass

            color = {"录制中": "#3fb950", "重连中": "#d29922",
                     "连接中": "#58a6ff", "出错": "#f85149"}.get(s["state"], "#8b949e")
            self.status.configure(text="● %s    %s" % (s["state"], rec.cur_path or rec.avi_path), fg=color)
            self.detail.configure(
                text="时长 %6.1f s | 帧 %6d | %5.2f fps | %7.2f MB | %5.1f KB/帧 | 卡顿 %d"
                     % (s["elapsed"], s["frames"], s["fps"], s["bytes"] / 1048576.0,
                        s["avr_kb"], s["gaps"]))
            extra = ""
            if rec.jpg_dir:
                extra = "逐帧图片 → %s" % rec.jpg_dir
            if s.get("error"):
                extra += "   [错误] %s" % s["error"]
            self.detail2.configure(text=extra)

            # 线程自己结束了（出错/停止）
            if not rec.is_alive() and self.btn_stop["state"] == "normal":
                self.btn_stop.configure(state="disabled")
                self.btn_start.configure(state="normal")
                if s["state"] == "出错":
                    self.status.configure(text="● 出错：%s" % s.get("error"), fg="#f85149")
                else:
                    self.status.configure(
                        text="● 已保存  %s  (%.1f MB)" % (rec.cur_path or rec.avi_path, s["bytes"] / 1048576.0),
                        fg="#3fb950")
        elif self._stop_ts == 0:
            self._stop_ts = 1

        self.after(200, self._tick)


if __name__ == "__main__":
    App().mainloop()
