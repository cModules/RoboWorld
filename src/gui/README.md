# GUI Module (Qt6 + OpenGL)

This directory will contain the Qt6-based 3D visualization and interactive programming environment.

## Planned Features
- OpenGL 4.5 viewport with scene graph rendering
- Drag-and-drop robot model loading
- Interactive joint jogging (slider panel)
- Node-based visual programming (future)
- Real-time collision highlighting

## Build
```bash
cmake .. -DBUILD_GUI=ON
```

## Dependencies
- Qt6 (Core, Widgets, OpenGLWidgets)
- OpenGL 4.5+
