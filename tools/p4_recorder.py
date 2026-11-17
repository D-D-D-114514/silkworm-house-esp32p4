#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
p4_recorder.py —— 从 ESP32-P4 的 /stream 抓 MJPEG 视频，保存到本地

为什么单独做这个工具：
  1. PC 的硬盘写 1.3MB/s 毫无压力，不受 P4 上 FATFS 簇分配开销的限制
  2. 不经过 SD 卡，省掉"换卡/读卡器/断电丢数据"这些麻烦
  3. 顺便可以给 P4 对时，解决文件名 1970 的问题

两种输出（可以同时开）：
  --avi     单个可播放的 MJPEG AVI，零转码、零质量损失（推荐，方便回放）
  --frames  把每一帧存成独立 .jpg（做 YOLO 数据集时最有用，可直接标注）

用法：
  python p4_recorder.py                       # 录到 Ctrl+C，存 out_YYYYmmdd_HHMMSS.avi
  python p4_recorder.py --seconds 600         # 录 10 分钟自动停
  python p4_recorder.py --frames              # 同时导出逐帧 jpg
  python p4_recorder.py --host 192.168.1.10 --sync-time   # 先给板子对时

依赖：只用 Python 标准库，不需要安装任何东西。
"""

import argparse
import datetime
import http.client
import os
import socket
import struct
import sys
import time

# ---------------------------------------------------------------- AVI 封装

class AviWriter:
    """手写 MJPEG AVI 容器。

    和 P4 固件里那套格式完全一致，但修正了两处：
      · avih.flags 一开始写 0，只有在 close 时真的写完 idx1 才置 AVIF_HASINDEX
        （固件旧版直接写 0x10，断电的文件会"声称有索引但没有"，严格播放器打不开）
      · strh 用 dwScale=us/帧、dwRate=1000000，帧率和 avih 保持一致
    """

    def __init__(self, path, width=1920, height=1080):
        self.path = path
        self.w, self.h = width, height
        self.fp = open(path, 'wb', buffering=256 * 1024)
        self.frames = 0
        self.max_frame = 0
        self.t_start = time.time()
        self.idx = []           # [(offset_in_movi, size), ...]

        f = self.fp
        f.write(b'RIFF')
        self.off_riff = f.tell(); f.write(struct.pack('<I', 0))
        f.write(b'AVI ')

        # LIST hdrl
        f.write(b'LIST'); f.write(struct.pack('<I', 4 + 64 + 12 + 64 + 48))
        f.write(b'hdrl')

        # avih
        f.write(b'avih'); f.write(struct.pack('<I', 56))
        self.off_uspf = f.tell();   f.write(struct.pack('<I', 40000))
        self.off_maxbps = f.tell(); f.write(struct.pack('<I', 0))
        f.write(struct.pack('<I', 0))                  # padding granularity
        self.off_flags = f.tell();  f.write(struct.pack('<I', 0))   # 先声明"无索引"
        self.off_nframes = f.tell(); f.write(struct.pack('<I', 0))
        f.write(struct.pack('<I', 0))                  # initial frames
        f.write(struct.pack('<I', 1))                  # streams
        f.write(struct.pack('<I', 0))                  # suggested buffer
        f.write(struct.pack('<I', width))
        f.write(struct.pack('<I', height))
        for _ in range(4):
            f.write(struct.pack('<I', 0))

        # LIST strl
        f.write(b'LIST'); f.write(struct.pack('<I', 4 + 64 + 48))
        f.write(b'strl')

        # strh
        f.write(b'strh'); f.write(struct.pack('<I', 56))
        f.write(b'vids'); f.write(b'MJPG')
        f.write(struct.pack('<I', 0))                  # flags
        f.write(struct.pack('<H', 0))                  # priority
        f.write(struct.pack('<H', 0))                  # language
        f.write(struct.pack('<I', 0))                  # initial frames
        self.off_scale = f.tell(); f.write(struct.pack('<I', 1))
        self.off_rate = f.tell();  f.write(struct.pack('<I', 25))
        f.write(struct.pack('<I', 0))                  # dwStart
        self.off_length = f.tell(); f.write(struct.pack('<I', 0))
        self.off_sug = f.tell();    f.write(struct.pack('<I', 0))
        f.write(struct.pack('<I', 0xFFFFFFFF))         # quality
        f.write(struct.pack('<I', 0))                  # sample size
        f.write(struct.pack('<H', 0)); f.write(struct.pack('<H', 0))
        f.write(struct.pack('<H', width)); f.write(struct.pack('<H', height))

        # strf
        f.write(b'strf'); f.write(struct.pack('<I', 40))
        f.write(struct.pack('<I', 40))
        f.write(struct.pack('<i', width)); f.write(struct.pack('<i', height))
        f.write(struct.pack('<H', 1))                  # planes
        f.write(struct.pack('<H', 24))                 # bitcount
        f.write(b'MJPG')
        f.write(struct.pack('<I', width * height * 3))
        for _ in range(4):
            f.write(struct.pack('<I', 0))

        # LIST movi
        f.write(b'LIST')
        self.off_movi = f.tell(); f.write(struct.pack('<I', 0))
        self.movi_pos = f.tell(); f.write(b'movi')
        self.movi_data = f.tell()          # movi 数据区起点（索引偏移的基准）

    def add(self, jpg):
        pos = self.fp.tell()
        self.fp.write(b'00dc')
        self.fp.write(struct.pack('<I', len(jpg)))
        self.fp.write(jpg)
        if len(jpg) & 1:
            self.fp.write(b'\x00')
        self.idx.append((pos - self.movi_data + 4, len(jpg)))
        self.frames += 1
        if len(jpg) > self.max_frame:
            self.max_frame = len(jpg)

    def _stats(self):
        elapsed = max(time.time() - self.t_start, 0.001)
        uspf = int(elapsed * 1_000_000 / max(self.frames, 1))
        uspf = max(1000, min(uspf, 1_000_000))
        return uspf, max(1, int(round(1_000_000.0 / uspf)))

    def sync(self):
        """录制中回填头部（不写 idx1，也不置 HASINDEX）。

        必要性：只在 close() 才回填的话，程序崩溃/断电后文件头还停在
        "0 帧 / 25fps" 的初值，几百 MB 的录像直接打不开。
        回填后即使没有索引，播放器顺序扫描也能正常播。
        """
        f = self.fp
        cur = f.tell()
        f.flush()
        uspf, fps = self._stats()

        def p32(off, val):
            f.seek(off)
            f.write(struct.pack('<I', val))

        p32(self.off_riff,    cur - 8)
        p32(self.off_nframes, self.frames)
        p32(self.off_uspf,    uspf)
        p32(self.off_maxbps,  self.max_frame * fps)
        p32(self.off_scale,   uspf)
        p32(self.off_rate,    1_000_000)
        p32(self.off_length,  self.frames)
        p32(self.off_sug,     self.max_frame)
        p32(self.off_movi,    cur - self.movi_data)
        f.seek(cur)
        f.flush()

    def close(self):
        f = self.fp
        self.sync()                         # 先把头部补到"到此刻为止有效"
        movi_end = f.tell()                 # movi 数据区结束（每帧已按 2 字节对齐，必为偶）

        # ---- idx1 ----
        f.write(b'idx1')
        f.write(struct.pack('<I', len(self.idx) * 16))
        for off, size in self.idx:
            f.write(b'00dc')
            f.write(struct.pack('<III', 0x10, off, size))
        total = f.tell()

        uspf, fps = self._stats()

        def p32(off, val):
            f.seek(off)
            f.write(struct.pack('<I', val))

        p32(self.off_riff,    total - 8)
        p32(self.off_nframes, self.frames)
        p32(self.off_uspf,    uspf)
        p32(self.off_maxbps,  self.max_frame * fps)
        p32(self.off_scale,   uspf)          # dwScale = us/帧
        p32(self.off_rate,    1_000_000)     # dwRate  = 1e6  → fps = 1e6/uspf
        p32(self.off_length,  self.frames)
        p32(self.off_sug,     self.max_frame)
        p32(self.off_movi,    movi_end - self.movi_data)   # movi 只算数据区
        p32(self.off_flags,   0x10)          # 索引确实写了，才置 AVIF_HASINDEX
        f.close()


# ---------------------------------------------------------------- MJPEG 抓流

class StreamReader:
    """按 multipart/x-mixed-replace 逐帧读出 JPEG。"""

    def __init__(self, host, port, path='/stream', timeout=10):
        self.host, self.port = host, port
        self.conn = http.client.HTTPConnection(host, port, timeout=timeout)
        self.conn.request('GET', path)
        self.resp = self.conn.getresponse()
        if self.resp.status != 200:
            raise RuntimeError('HTTP %d %s' % (self.resp.status, self.resp.reason))
        self.boundary = None
        ctype = self.resp.getheader('Content-Type') or ''
        if 'boundary=' in ctype:
            self.boundary = ctype.split('boundary=')[-1].strip().strip('"')
        self.buf = b''

    # --- 底层读取：http.client 会自己处理 chunked 编码，这里只处理"读够 N 字节"
    def _fill(self, want):
        while len(self.buf) < want:
            chunk = self.resp.read(65536)
            if not chunk:
                raise EOFError('连接被关闭')
            self.buf += chunk

    def _read_until(self, delim, limit=65536):
        while True:
            i = self.buf.find(delim)
            if i >= 0:
                out = self.buf[:i]
                self.buf = self.buf[i + len(delim):]
                return out
            if len(self.buf) > limit:
                raise RuntimeError('头部过长，可能不是 MJPEG 流')
            chunk = self.resp.read(65536)
            if not chunk:
                raise EOFError('连接被关闭')
            self.buf += chunk

    def next_frame(self):
        """阻塞到下一帧，返回 JPEG 字节"""
        # 跳过第一段边界
        while True:
            line = self._read_until(b'\r\n', limit=4096)
            if line.startswith(b'--'):
                break

        length = None
        while True:
            line = self._read_until(b'\r\n', limit=8192)
            if not line:
                break
            if line.lower().startswith(b'content-length:'):
                length = int(line.split(b':', 1)[1].strip())

        if length is None:
            raise RuntimeError('帧头里没有 Content-Length')
        self._fill(length)
        jpg = self.buf[:length]
        self.buf = self.buf[length:]
        # 丢掉帧后跟着的 \r\n
        self._read_until(b'\r\n', limit=64)
        return bytes(jpg)

    def close(self):
        try:
            self.conn.close()
        except Exception:
            pass


# ---------------------------------------------------------------- 对时

def sync_time(host, port):
    """把 PC 的当前时间推给 P4，否则它录出来的文件名是 1970 年"""
    try:
        epoch = int(time.time())
        c = http.client.HTTPConnection(host, port, timeout=5)
        c.request('GET', '/time?epoch=%d' % epoch)
        r = c.getresponse()
        body = r.read().decode('utf-8', 'replace').strip()
        ok = (r.status == 200)
        c.close()
        print('[对时] %s  epoch=%d  %s' % ('OK ' if ok else '失败', epoch, body[:80]))
        return ok
    except Exception as e:
        print('[对时] 失败: %s' % e)
        return False


# ---------------------------------------------------------------- 主程序

def human(n):
    for unit in ('B', 'KB', 'MB', 'GB'):
        if n < 1024 or unit == 'GB':
            return '%.1f %s' % (n, unit)
        n /= 1024.0


def main():
    ap = argparse.ArgumentParser(description='ESP32-P4 MJPEG 抓流录制器')
    ap.add_argument('--host', default='192.168.1.10')
    ap.add_argument('--port', type=int, default=80)
    ap.add_argument('--seconds', type=float, default=0, help='录制时长，0=直到 Ctrl+C')
    ap.add_argument('--out', default='.', help='输出目录')
    ap.add_argument('--name', default=None, help='输出文件名（不含扩展名）')
    ap.add_argument('--frames', action='store_true', help='同时把每帧存成 jpg')
    ap.add_argument('--no-avi', action='store_true', help='只存 jpg，不生成 avi')
    ap.add_argument('--every', type=int, default=0, help='每 N 帧存一张 jpg（0=全部）')
    ap.add_argument('--sync-time', action='store_true', help='开始前给 P4 对时')
    ap.add_argument('--resize', default=None, help='AVI 里声明的分辨率，如 1920x1080（只影响元数据）')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    if args.sync_time:
        sync_time(args.host, args.port)

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    base = args.name or ('p4_' + stamp)
    avi_path = os.path.join(args.out, base + '.avi')
    jpg_dir = os.path.join(args.out, base + '_frames') if args.frames else None
    if jpg_dir:
        os.makedirs(jpg_dir, exist_ok=True)

    w, h = 1920, 1080
    if args.resize and 'x' in args.resize:
        w, h = [int(x) for x in args.resize.lower().split('x')]

    print('[连接] %s:%d/stream ...' % (args.host, args.port))
    reader = StreamReader(args.host, args.port)
    print('[连接] 成功，boundary=%s' % reader.boundary)

    avi = None if args.no_avi else AviWriter(avi_path, w, h)
    if avi:
        print('[输出] %s' % avi_path)
    if jpg_dir:
        print('[输出] %s/  （逐帧 jpg）' % jpg_dir)

    t0 = time.time()
    last_report = t0
    last_frame_t = t0
    gaps = 0
    n = 0
    total_bytes = 0
    worst_gap = 0.0

    try:
        while True:
            if args.seconds and (time.time() - t0) >= args.seconds:
                print('\n[结束] 到达设定时长')
                break

            try:
                jpg = reader.next_frame()
            except (EOFError, socket.timeout, ConnectionError) as e:
                print('\n[断开] %s，1 秒后重连...' % e)
                reader.close()
                time.sleep(1.0)
                reader = StreamReader(args.host, args.port)
                continue

            now = time.time()
            dt = now - last_frame_t
            last_frame_t = now
            if dt > 0.35:                    # 正常情况下 0.1s 一帧
                gaps += 1
                worst_gap = max(worst_gap, dt)

            n += 1
            total_bytes += len(jpg)
            if avi:
                avi.add(jpg)
                if avi.frames % 10 == 0:      # 每约 1 秒回填一次，崩溃也不丢整段
                    avi.sync()

            if jpg_dir and (args.every <= 1 or n % args.every == 0):
                with open(os.path.join(jpg_dir, 'frame_%06d.jpg' % n), 'wb') as f:
                    f.write(jpg)

            if now - last_report >= 2.0:
                el = now - t0
                print('  %5.1f s | %5d 帧 | %5.2f fps | %7.2f MB | %6.1f KB/帧 | 卡顿 %d'
                      % (el, n, n / el, total_bytes / 1048576.0,
                         total_bytes / max(n, 1) / 1024.0, gaps))
                last_report = now

    except KeyboardInterrupt:
        print('\n[停止] Ctrl+C')

    finally:
        el = max(time.time() - t0, 0.001)
        print('\n================ 统计 ================')
        print('时长      : %.1f s' % el)
        print('帧数      : %d' % n)
        print('平均帧率  : %.2f fps' % (n / el))
        print('总字节    : %s' % human(total_bytes))
        print('平均帧大小: %.1f KB' % (total_bytes / max(n, 1) / 1024.0))
        print('卡顿次数  : %d（最长 %.2f s）' % (gaps, worst_gap))
        if avi and n:
            avi.close()
            print('已保存    : %s（%s）' % (avi_path, human(os.path.getsize(avi_path))))
        reader.close()


if __name__ == '__main__':
    main()
