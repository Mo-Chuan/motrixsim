# 🌐 Web Viewer

Use the MotrixSim Web Viewer to open models, run simulations, and inspect objects in your browser.

## Open the viewer

Open <https://motrix.motphys.com/> in a browser with WebAssembly support. Identify the layout by the **Files** panel. The public site currently shows a file tree on the left. The newer layout shown below has a **Files** panel below the viewport.

### Load Spot on the public site

1. In **Files** on the left, expand **Online** and then **boston_dynamics_spot**.
2. Click **scene.xml** once. Spot appears in the central viewport after the scene loads.
3. Use the left toolbar to play, pause, or step the simulation. The separate **Joint** and **Actuator** panels are on the right. Display settings are in **Scenes** on the left.

## Load a built-in example in the newer layout

1. Expand **Online** in the directory tree of the **Files** panel.
2. Click the `boston_dynamics_spot` folder.
3. Double-click `scene.xml` in the file list. When **Loading resources** finishes, the Spot model appears in the viewport.

```{figure} /_static/images/web_viewer/current/master-overview.jpg
:alt: Spot in the central viewport, with Files at the bottom and inspection panels on the right
:width: 100%

A loaded model in the Web Viewer.
```

```{figure} /_static/images/web_viewer/current/master-files.jpg
:alt: Files shows the Online directory on the left, scene.xml in the middle, and file details on the right
:width: 100%

Double-click a scene file in the middle list to load it. A single click selects the file and shows its details.
```

Drag the dividers around the file list to give file names more room. Use **Search** to find files in the current directory, the filter button to narrow the list by model type, or the path above the list to navigate to a parent folder.

## Load a local model

Keep the scene file and all referenced meshes and textures in the same folder structure they use on disk. For example, if `scene.xml` references files in `meshes/`, include that directory when you drag the folder into the viewer.

```text
my_robot/
├── scene.xml
├── meshes/
└── textures/
```

1. Drag the entire model folder from your file manager into the viewer.
2. In **Files**, expand **Customize** and open the imported folder.
3. Double-click a scene file in the file list. Scene files use the `.xml`, `.urdf`, or `.json` extension.
4. Wait for **Loading resources** to finish and check that your model appears in the viewport.

**Online** contains the built-in examples. **Customize** contains the files you add to the current browser session. If your files disappear after a refresh, drag in the model folder again. If **Remote** appears, use it for remote resources you can access.

If the viewer reports a missing mesh or texture, check that the imported folder contains the file at the relative path specified in the scene. Add the missing files and drag the complete folder in again.

## Control the simulation

-   Click **Play** on the left toolbar to start the simulation. Click **Pause** to keep the current state.
-   While paused, click **Step** to advance one frame.
-   Click **Stop** to reset the simulation and return to editing. **Reset** restores the initial scene state. **Reload** reads the model file and its assets again.

Click the viewport to give it keyboard focus, then use these shortcuts. If the browser intercepts a shortcut, use the corresponding toolbar button.

| Key | Action |
| --- | --- |
| `Space` | Toggle playback |
| `F10` | Step one frame |
| `Ctrl+E` | Reset the scene |
| `Ctrl+R` | Reload the model. Use the reload button if your browser refreshes the page instead |
| `F11` | Toggle fullscreen where supported |

## Control the camera

Drag with the left mouse button to orbit, drag with the right button to pan, and scroll to zoom.

Use the camera selector at the top left of the viewport to choose a camera. The selector displays the active camera, such as **Perspective**. Open the gear menu beside it for **Display Mode**, **Camera Settings**, **Visibility**, and **Gizmos**.

```{figure} /_static/images/web_viewer/current/master-viewport-menu.jpg
:alt: The viewport gear menu shows camera, visibility, and gizmo settings
:width: 100%

The gear menu is to the left of the camera selector.
```

If the application enables physics drag, hold `Ctrl` and drag an object with the left mouse button.

```{video} /_static/videos/physics_drag.mp4
:poster: /_static/images/poster/physics_drag.jpg
:caption: Hold Ctrl and drag an object
:playsinline:
:width: 100%
```

## Inspect joints and actuators

1. Open **Inspector** on the right.
2. Select a joint name in the **Joint** group. The viewer highlights the corresponding part of the model.
3. Open **Properties** to see the joint's type, range, and simulation state. Use the search field to find a property.

```{figure} /_static/images/web_viewer/current/master-inspector.jpg
:alt: Inspector shows the Joint and Actuator groups for the Spot model
:width: 100%

Choose a joint in **Joint** or inspect an actuator and its control value in **Actuator**.
```

```{figure} /_static/images/web_viewer/current/master-properties.jpg
:alt: A highlighted joint in the viewport with its properties and state in the Properties panel
:width: 100%

**Properties** shows details for the selected joint.
```

Play the simulation to observe position and velocity values. To keep a joint or actuator at hand, click its star in **Inspector**, then open **Favorites**. Use the tab bar arrows or widen the panel if a tab is out of view.

## Adjust physics and rendering options

Open **Options** on the right. Use **Physics** to view keyframes, dynamics, gravity, and actuator settings. Use **Rendering** or **Lighting / Environment** to adjust the scene's appearance.

```{figure} /_static/images/web_viewer/current/master-options.jpg
:alt: The Physics category in Options with Keyframes, Dynamics, Gravity, and Actuators
:width: 100%

Scroll in **Physics** to find simulation settings.
```

## See also

If the viewer has a **Help** button, use it to open this guide.

-   [Installation](installation.md)
-   [Hello MotrixSim](hello_motrixsim.md)
-   [Interactive Viewer](../main_function/managed_viewer.md)
-   [Renderer](../main_function/render.md)
