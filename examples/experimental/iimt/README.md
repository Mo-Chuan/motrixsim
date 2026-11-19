# IIMT Auto Store Demo

This directory contains the Python task-style implementation of the IIMT auto store demo.

It is different from `iimt_auto_store_lite.py`:

-   `iimt_auto_store_lite.py`
    -   lightweight interactive scene demo
    -   manual base control
    -   manual arm target adjustment
-   `iimt_auto_store_task.py`
    -   task-oriented demo
    -   includes base waypoint navigation
    -   includes arm IK motion and command flow for pick-and-place

## Entry

Run the task demo from the `iimt` directory:

```bash
cd motrixsim-python/motrixsim-docs/examples/experimental/iimt
uv run python iimt_auto_store_task.py
```

## What It Does

The demo loads the IIMT store scene, lets you select a shelf item, drives the mobile base to the corresponding shelf waypoint, uses the arm to grasp the item, then drives back toward the basket/table area and places the item.

The implementation is split into several files:

-   `common.py`
    -   shared math helpers, selection helpers, constants
-   `transporter_controller.py`
    -   base waypoint navigation and PID-based low-level control
-   `arm_controller.py`
    -   arm waypoint tracking and IK solving
-   `task_flow.py`
    -   task command stack and high-level task sequence
-   `app.py`
    -   rendering, input handling, and main loop

## Controls

### `iimt_auto_store_lite.py`

-   Left click
    -   select and highlight a shelf item
-   Left drag
    -   orbit the system camera
-   Right drag
    -   pan the system camera
-   `1`
    -   switch to system camera
-   `2`
    -   switch to scene camera `aim`
-   `W / S / A / D`
    -   manually drive the mobile base
-   `Space`
    -   move the arm target above the selected item
-   Arrow keys
    -   move the arm target in x / y
-   `U / J`
    -   move the arm target in z
-   `Q / E`
    -   rotate the arm target around z
-   `Z / X`
    -   rotate the arm target around x
-   `C / O`
    -   close / open the gripper
-   `R`
    -   reset the arm target
-   `P`
    -   reset the whole scene
-   `Esc`
    -   quit

### `iimt_auto_store_task.py`

-   Right click
    -   select a shelf item in front of the camera
-   Left drag
    -   rotate the view
-   Right drag
    -   move the camera position
-   `1`
    -   switch to system camera
-   `2`
    -   switch to scene camera `aim`
-   `W / S / A / D`
    -   manually drive the base when no task command is running
-   `Space`
    -   start the pick-and-place flow for the selected item
-   `T`
    -   run the initialization flow
-   `Y`
    -   run the finish flow
-   `R`
    -   reset the arm and clear task commands
-   `P`
    -   reset the whole scene
-   `Esc`
    -   quit

## Notes

-   The base uses predefined shelf waypoints.
-   The arm uses IK and waypoint-based motion.
-   This demo is still under active adjustment; waypoint positions and grasp behavior may need further tuning depending on the selected item and scene state.
