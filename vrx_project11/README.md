# vrx_project11

To regenerate the wamv urdf configuration, run the following in this package's root folder.

```bash
ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=`pwd`/config/wamv_config/component_config.yaml wamv_target:=`pwd`/urdf/wamv.urdf
```
