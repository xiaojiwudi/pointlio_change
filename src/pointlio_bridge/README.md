
## 地图保存指令

```sh
ros2 service call /save_map std_srvs/srv/Trigger {}
```

如果成功，会返回 [success]: [true] 和保存路径信息，地图文件会保存到节点参数 [map_save_dir] 指定的目录（默认 /tmp/lego_maps），文件名前缀为 map_save_prefix（默认 lego_map_mid360）。

