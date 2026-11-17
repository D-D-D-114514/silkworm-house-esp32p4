idf_component_register(
    SRCS 
        "main.c"
        "camera_task.c"
        "video_record_task.c"
        "http_server_task.c"
        "uart_comm_task.c"
        "log_task.c"
        "ai_policy.c"
        "ethernet_init.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "index.html"
)
