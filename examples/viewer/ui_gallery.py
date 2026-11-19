# Copyright (C) 2020-2026 Motphys Technology Co., Ltd. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""MotrixSim UI Gallery for Viewer blueprints, controls, images, and lifecycle APIs."""

import numpy as np

from motrixsim import SceneData, load_model, run, step, ui
from motrixsim.render import InvalidArgumentError, RenderApp

TARGET_VIEW_TITLE = "Free Camera"
LOOKUP_BY_REGEX = "Title regex"
LOOKUP_BY_EXACT_TITLE = "Exact title"
CONTROL_MODES = ["Position", "Velocity", "Torque"]
GEOMETRY_GROUPS = ["Group 0", "Group 1", "Group 2", "Group 3"]
DYNAMIC_SELECTION_ITEMS = ["Option A", "Option B", "Option C", "Option D"]


class UiGalleryController:
    def __init__(self, move_slider):
        self._move_slider = move_slider
        self.render = None
        self.gallery_view = None
        self.lookup_frame = None
        self.demo_frame = None
        self.demo_frame_extended = False
        self.frame_select_state = None
        self.include_query = r"Free Camera"
        self.exclude_query = ""
        self.lookup_mode = LOOKUP_BY_REGEX
        self.selected_title = TARGET_VIEW_TITLE
        self.control_mode = CONTROL_MODES[0]
        self.geometry_groups = list(GEOMETRY_GROUPS)
        self.verbose = True
        self.button_state = None
        self.move_slider_query_state = None
        self.checkbox_state = None
        self.text_input_state = None
        self.select_state = None
        self.radio_group_state = None
        self.checkbox_group_state = None
        self.toggle_group_state = None
        self.options_select = None
        self.options_radio_group = None
        self.options_checkbox_group = None
        self.options_toggle_group = None

    def bind(self, render, gallery_view, lookup_frame, demo_frame):
        self.render = render
        self.gallery_view = gallery_view
        self.lookup_frame = lookup_frame
        self.demo_frame = demo_frame

    def demo_button_clicked(self):
        self._log("Button clicked")

    def move_slider_from_callback(self):
        self._move_slider()
        self._log("moved slider from callback")

    def set_demo_checkbox(self, value: bool):
        self._log(f"checkbox value: {value}")

    def set_demo_text(self, value: str):
        self._log(f"text value: {value!r}")

    def submit_demo_text(self, value: str):
        self._log(f"text submitted: {value!r}")

    def set_demo_combo(self, value: str):
        self._log(f"combo value: {value}")

    def poll_demo_controls(self):
        if self.move_slider_query_state is not None and self.move_slider_query_state.is_clicked():
            self._move_slider()
            self._log("moved slider from query after sync")
        if self.button_state is not None and self.button_state.is_clicked():
            self._log("query: button clicked")
        if self.checkbox_state is not None and self.checkbox_state.is_changed():
            self._log(f"query: checkbox value: {self.checkbox_state.value}")
        if self.text_input_state is not None:
            if self.text_input_state.is_changed():
                self._log(f"query: text value: {self.text_input_state.value!r}")
            if self.text_input_state.is_submitted():
                self._log(f"query: text submitted: {self.text_input_state.value!r}")
        if self.select_state is not None and self.select_state.is_changed():
            self._log(f"query: combo value: {self.select_state.value}")
        if self.radio_group_state is not None and self.radio_group_state.is_changed():
            self._log(f"query: radio value: {self.radio_group_state.value}")
        if self.checkbox_group_state is not None and self.checkbox_group_state.is_changed():
            self._log(f"query: checkbox group value: {self.checkbox_group_state.value!r}")
        if self.toggle_group_state is not None and self.toggle_group_state.is_changed():
            self._log(f"query: toggle group value: {self.toggle_group_state.value!r}")

        for label, state in (
            ("Update choices / Select", self.options_select),
            ("Update choices / RadioGroup", self.options_radio_group),
            ("Update choices / CheckboxGroup", self.options_checkbox_group),
            ("Update choices / ToggleGroup", self.options_toggle_group),
            ("Frame / Select", self.frame_select_state),
        ):
            if state is not None and state.is_changed():
                self._log(f"query after sync: {label}, value={state.value!r}, options={state.options!r}")

    def build_demo_frame(self):
        with ui.Column():
            ui.Text("Frame base node")
            items = DYNAMIC_SELECTION_ITEMS if self.demo_frame_extended else DYNAMIC_SELECTION_ITEMS[:2]
            self.frame_select_state = ui.Select(
                "Options set during rebuild",
                options=items,
                value=items[-1],
                on_change=lambda value: self._log(f"callback: Frame / Select, value={value!r}"),
            )
            self._log(
                f"Frame builder: extended={self.demo_frame_extended}, requested options={items!r}, value={items[-1]!r}"
            )
            if self.demo_frame_extended:
                with ui.Row():
                    ui.Text("Conditional node A")
                    ui.Text("Conditional node B")

    def set_demo_frame_extended(self, value: bool):
        self.demo_frame_extended = value
        self._require_render().rebuild(self._require_demo_frame())
        self._log(f"requested Frame rebuild: extended={value}")

    def set_choice_options_extended(self, value: bool):
        items = DYNAMIC_SELECTION_ITEMS if value else DYNAMIC_SELECTION_ITEMS[:2]
        self.options_select.set_options(items)
        self.options_radio_group.set_options(items)
        self.options_checkbox_group.set_options(items)
        self.options_toggle_group.set_options(items)
        self._log(f"requested choice options: {items!r}")

    def select_last_choice_values(self):
        combo_options = self.options_select.options
        radio_options = self.options_radio_group.options
        checkbox_options = self.options_checkbox_group.options
        toggle_options = self.options_toggle_group.options
        self.options_select.set_value(combo_options[-1])
        self.options_radio_group.set_value(radio_options[-1])
        self.options_checkbox_group.set_value([checkbox_options[-1]])
        self.options_toggle_group.set_value([toggle_options[-1]])
        self._log("requested all choices to select their last value")

    def build_lookup_panel(self):
        with ui.Column():
            if self.lookup_mode == LOOKUP_BY_REGEX:
                self.build_title_controls()
            else:
                self.build_exact_title_controls()

    def build_title_controls(self):
        ui.Text("Select Views with full-match title regular expressions")
        ui.TextInput(
            "Include title regex",
            value=self.include_query,
            on_change=self.set_include_query,
        )
        ui.TextInput(
            "Exclude title regex",
            value=self.exclude_query,
            on_change=self.set_exclude_query,
        )
        with ui.Row():
            ui.Button("Hide titles", on_click=self.hide_titles)
            ui.Button("Show titles", on_click=self.show_titles)

    def build_exact_title_controls(self):
        ui.Text("Apply an action to one known live or hidden final title")
        ui.TextInput(
            "Final title",
            value=self.selected_title,
            on_change=self.set_selected_title,
        )
        with ui.Row():
            ui.Button("Hide title", on_click=self.hide_selected_title)
            ui.Button("Show title", on_click=self.show_selected_title)

    def set_include_query(self, value: str):
        self.include_query = value
        self._log(f"include regex: {value!r}")

    def set_exclude_query(self, value: str):
        self.exclude_query = value
        self._log(f"exclude regex: {value!r}")

    def set_lookup_mode(self, value: str):
        if value == self.lookup_mode:
            return
        self.lookup_mode = value
        self._log(f"lookup mode: {value}")
        self._require_render().rebuild(self._require_lookup_frame())

    def set_selected_title(self, value: str):
        self.selected_title = value
        self._log(f"selected final title: {value}")

    def set_control_mode(self, value: str):
        self.control_mode = value
        self._log(f"control mode: {value}")

    def set_geometry_groups(self, values: list[str]):
        self.geometry_groups = values
        self._log(f"geometry groups: {values!r}")

    def set_verbose(self, value: bool):
        self.verbose = value
        print(f"[UI Gallery] action logging: {value}")

    def hide_selection(self):
        if self.lookup_mode == LOOKUP_BY_REGEX:
            self.hide_titles()
        else:
            self.hide_selected_title()

    def show_selection(self):
        if self.lookup_mode == LOOKUP_BY_REGEX:
            self.show_titles()
        else:
            self.show_selected_title()

    def hide_titles(self):
        render = self._require_render()
        include, exclude = self._title_selection()
        self._log(f"hide by title: include={include!r}, exclude={exclude!r}")
        try:
            render.hide(include=include, exclude=exclude)
        except InvalidArgumentError as error:
            print(f"[UI Gallery] invalid title regex: {error}")

    def show_titles(self):
        render = self._require_render()
        include, exclude = self._title_selection()
        self._log(f"show by title: include={include!r}, exclude={exclude!r}")
        try:
            render.show(include=include, exclude=exclude)
        except InvalidArgumentError as error:
            print(f"[UI Gallery] invalid title regex: {error}")

    def hide_selected_title(self):
        self._log(f"hide exact title: {self.selected_title}")
        self._require_render().hide(title=self.selected_title)

    def show_selected_title(self):
        self._log(f"show exact title: {self.selected_title}")
        self._require_render().show(title=self.selected_title)

    def hide_gallery(self):
        render, gallery_view = self._gallery_target()
        self._log("hide UI Gallery using its Python declaration")
        render.hide(gallery_view)

    def show_gallery(self):
        render, gallery_view = self._gallery_target()
        self._log("show UI Gallery using its Python declaration")
        render.show(gallery_view)

    def _title_selection(self):
        include = [self.include_query] if self.include_query else None
        exclude = [self.exclude_query] if self.exclude_query else []
        return include, exclude

    def _gallery_target(self):
        render = self._require_render()
        if self.gallery_view is None:
            raise RuntimeError("UI Gallery controller is not bound to its Python UI View")
        return render, self.gallery_view

    def _require_render(self):
        if self.render is None:
            raise RuntimeError("UI Gallery controller is not bound to a RenderApp")
        return self.render

    def _require_lookup_frame(self):
        if self.lookup_frame is None:
            raise RuntimeError("UI Gallery controller is not bound to its lookup Frame")
        return self.lookup_frame

    def _require_demo_frame(self):
        if self.demo_frame is None:
            raise RuntimeError("UI Gallery controller is not bound to its demo Frame")
        return self.demo_frame

    def _log(self, message: str):
        if self.verbose:
            print(f"[UI Gallery] {message}", flush=True)


# tag::ui_gallery[]
def show_code(source: str):
    ui.Text(source)


def build_python_views(controller):
    with ui.View(title="UI Gallery") as gallery:
        with ui.Column():
            ui.Text("MotrixSim UI Gallery")
            ui.Text("Each section shows minimal Python code followed by the live result.")
            with ui.Column():
                with ui.Column():
                    with ui.Section("ui.View"):
                        show_code('with ui.View(title="UI Gallery") as view:\n    with ui.Column():\n        ...')
                        ui.Text("This entire panel is the live ui.View result.")

                    with ui.Section("ui.Text"):
                        show_code('ui.Text("Hello from MotrixSim")')
                        ui.Text("Hello from MotrixSim")

                    with ui.Section("ui.Button"):
                        show_code('button = ui.Button("Click me", on_click=on_click)')
                        controller.button_state = ui.Button(
                            "Click me",
                            on_click=controller.demo_button_clicked,
                        )

                    with ui.Section("ui.Checkbox"):
                        show_code('checkbox = ui.Checkbox("Enabled", value=False, on_change=on_change)')
                        controller.checkbox_state = ui.Checkbox(
                            "Enabled",
                            value=False,
                            on_change=controller.set_demo_checkbox,
                        )

                    with ui.Section("ui.TextInput"):
                        show_code(
                            'text_input = ui.TextInput("Label", value="Scene", '
                            "on_change=on_change, on_submit=on_submit)"
                        )
                        controller.text_input_state = ui.TextInput(
                            "Label",
                            value="Scene",
                            on_change=controller.set_demo_text,
                            on_submit=controller.submit_demo_text,
                        )

                    with ui.Section("ui.Select"):
                        show_code('select = ui.Select("Preset", options=items, value=items[0], on_change=on_change)')
                        controller.select_state = ui.Select(
                            "Control preset",
                            options=CONTROL_MODES,
                            value=CONTROL_MODES[0],
                            on_change=controller.set_demo_combo,
                        )

                    with ui.Section("ui.RadioGroup"):
                        show_code(
                            'radio_group = ui.RadioGroup("Mode", options=items, '
                            'value=items[0], direction="horizontal", ...)'
                        )
                        controller.radio_group_state = ui.RadioGroup(
                            "Control mode",
                            options=CONTROL_MODES,
                            value=controller.control_mode,
                            direction="horizontal",
                            on_change=controller.set_control_mode,
                        )

                    with ui.Section("ui.CheckboxGroup"):
                        show_code(
                            'ui.CheckboxGroup("One row", options, columns=None, ...)\n'
                            'ui.CheckboxGroup("1 column", options, columns=1, ...)\n'
                            'ui.CheckboxGroup("2 columns", options, columns=2, ...)\n'
                            'ui.CheckboxGroup("3 columns", options, columns=3, ...)'
                        )
                        for columns, label in (
                            (None, "One row"),
                            (1, "1 column"),
                            (2, "2 columns"),
                            (3, "3 columns"),
                        ):
                            state = ui.CheckboxGroup(
                                label,
                                options=GEOMETRY_GROUPS,
                                value=controller.geometry_groups,
                                columns=columns,
                                on_change=controller.set_geometry_groups,
                            )
                            if columns == 1:
                                controller.checkbox_group_state = state

                    with ui.Section("ui.ToggleGroup"):
                        show_code(
                            'ui.ToggleGroup("One row", options, columns=None, ...)\n'
                            'ui.ToggleGroup("1 column", options, columns=1, ...)\n'
                            'ui.ToggleGroup("2 columns", options, columns=2, ...)\n'
                            'ui.ToggleGroup("3 columns", options, columns=3, ...)'
                        )
                        for columns, label in (
                            (None, "One row"),
                            (1, "1 column"),
                            (2, "2 columns"),
                            (3, "3 columns"),
                        ):
                            state = ui.ToggleGroup(
                                label,
                                options=GEOMETRY_GROUPS,
                                value=controller.geometry_groups,
                                columns=columns,
                                on_change=controller.set_geometry_groups,
                            )
                            if columns == 2:
                                controller.toggle_group_state = state

                    with ui.Section("Update choice options and values"):
                        show_code(
                            "select.set_options(items)\n"
                            "radio_group.set_options(items)\n"
                            "checkbox_group.set_options(items)\n"
                            "toggle_group.set_options(items)\n"
                            "# Applied together by the next render.sync(data)\n"
                            "select.set_value(items[-1])"
                        )
                        items = DYNAMIC_SELECTION_ITEMS[:2]
                        controller.options_select = ui.Select(
                            "Select",
                            options=items,
                            on_change=lambda value: controller._log(
                                f"callback: Update choices / Select, value={value!r}"
                            ),
                        )
                        controller.options_radio_group = ui.RadioGroup(
                            "RadioGroup",
                            options=items,
                            on_change=lambda value: controller._log(
                                f"callback: Update choices / RadioGroup, value={value!r}"
                            ),
                        )
                        controller.options_checkbox_group = ui.CheckboxGroup(
                            "CheckboxGroup",
                            options=items,
                            on_change=lambda value: controller._log(
                                f"callback: Update choices / CheckboxGroup, value={value!r}"
                            ),
                        )
                        controller.options_toggle_group = ui.ToggleGroup(
                            "ToggleGroup",
                            options=items,
                            on_change=lambda value: controller._log(
                                f"callback: Update choices / ToggleGroup, value={value!r}"
                            ),
                        )
                        ui.Checkbox(
                            "Use extended choice options",
                            value=False,
                            on_change=controller.set_choice_options_extended,
                        )
                        ui.Button(
                            "Select last value",
                            on_click=controller.select_last_choice_values,
                        )
                        ui.Text("No Frame rebuild is needed for Choice property updates.")

                    with ui.Section("Query control state after sync"):
                        show_code(
                            "render.sync(data)\n"
                            "if button.is_clicked():\n"
                            "    ...\n"
                            "if checkbox.is_changed():\n"
                            "    use(checkbox.value)\n"
                            "if text_input.is_changed():\n"
                            "    use(text_input.value)\n"
                            "if text_input.is_submitted():\n"
                            "    submit(text_input.value)\n"
                            "if select.is_changed():\n"
                            "    use(select.value)\n"
                            "if radio_group.is_changed():\n"
                            "    use(radio_group.value)\n"
                            "if toggle_group.is_changed():\n"
                            "    use(toggle_group.value)"
                        )
                        ui.Text("The gallery polls these states once after every render.sync(data).")

                    with ui.Section("ui.Row"):
                        show_code("with ui.Row():\n    ui.Button(...)\n    ui.Button(...)")
                        with ui.Row():
                            ui.Button("First", on_click=controller.demo_button_clicked)
                            ui.Button("Second", on_click=controller.demo_button_clicked)

                    with ui.Section("ui.Column"):
                        show_code("with ui.Column():\n    ui.Text(...)\n    ui.Text(...)")
                        with ui.Column():
                            ui.Text("Column item A")
                            ui.Text("Column item B")

                    with ui.Section("ui.Section"):
                        show_code('with ui.Section("Nested section"):\n    ui.Text(...)')
                        with ui.Section("Nested section"):
                            ui.Text("Section content")

                    with ui.Section("ui.ScrollArea"):
                        show_code("with ui.ScrollArea(max_height=160):\n    with ui.Column():\n        ...")
                        with ui.ScrollArea(max_height=160):
                            with ui.Column():
                                for index in range(1, 13):
                                    ui.Text(f"Scrollable row {index}")

                    with ui.Section("ui.Frame"):
                        show_code(
                            "def build():\n"
                            "    with ui.Column():\n"
                            "        choice = ui.Select('Choice', options=['Preparing'])\n"
                            "        choice.set_options(items, value=items[-1])\n"
                            "        if expanded:\n"
                            "            ui.Text('Conditional node')\n"
                            "\n"
                            "frame = ui.Frame(build_fn=build)\n"
                            "expanded = True\n"
                            "render.rebuild(frame)"
                        )
                        ui.Checkbox(
                            "Rebuild with extended Frame options",
                            value=False,
                            on_change=controller.set_demo_frame_extended,
                        )
                        demo_frame = ui.Frame(build_fn=controller.build_demo_frame)

                    with ui.Section("ui.Image"):
                        show_code("preview = ui.Image(width=320); preview.set_image(image)")
                        ui.Text("See the live animated result in the UI Image Gallery View.")

    with ui.View(title="Viewer Lifecycle") as lifecycle:
        with ui.Column():
            ui.Text("Viewer lifecycle actions are separate from the widget gallery.")
            with ui.Section("Current selection"):
                ui.Select(
                    "Lookup by",
                    options=[LOOKUP_BY_REGEX, LOOKUP_BY_EXACT_TITLE],
                    value=LOOKUP_BY_REGEX,
                    on_change=controller.set_lookup_mode,
                )
                lookup_frame = ui.Frame(build_fn=controller.build_lookup_panel)
            with ui.Section("Python UI declaration"):
                ui.Text("Rebuild the Gallery Frame, then hide this View or close its tab")
                ui.Text("Show restores the same Python declaration and its latest Frame content")
                with ui.Row():
                    ui.Button("Hide UI Gallery", on_click=controller.hide_gallery)
                    ui.Button("Show UI Gallery", on_click=controller.show_gallery)
            with ui.Section("Diagnostics"):
                ui.Checkbox(
                    "Log actions",
                    value=controller.verbose,
                    on_change=controller.set_verbose,
                )

    return gallery, lifecycle, lookup_frame, demo_frame


def build_simulation_control_view(controller):
    with ui.View(title="Simulation Control") as simulation_control:
        with ui.Column():
            ui.Text("Drive the scene slider through callback or frame-aligned query input.")
            show_code(
                "force = 10\n"
                "def move_slider():\n"
                "    nonlocal force\n"
                "    actuator.set_ctrl(data, force)\n"
                "    force = -force"
            )
            with ui.Section("Callback"):
                show_code('ui.Button("Move slider", on_click=move_slider)')
                ui.Button(
                    "Move slider (callback)",
                    on_click=controller.move_slider_from_callback,
                )
            with ui.Section("Query after sync"):
                show_code(
                    'move_button = ui.Button("Move slider")\n'
                    "\n"
                    "render.sync(data)\n"
                    "if move_button.is_clicked():\n"
                    "    move_slider()"
                )
                controller.move_slider_query_state = ui.Button("Move slider (query)")
    return simulation_control


# end::ui_gallery[]


def build_layout_usage_view():
    with ui.View(title="Layout Usage") as layout_usage:
        with ui.Column():
            ui.Text("ViewerBlueprint containers compose RenderApp-supported Views.")
            with ui.Section("Horizontal, Vertical, and Tabs"):
                show_code(
                    "blueprint = ui.Horizontal([\n"
                    "    free_camera,\n"
                    "    simulation_control,\n"
                    "    ui.Vertical([\n"
                    "        layout_usage,\n"
                    '        ui.Options(title="Options"),\n'
                    "    ]),\n"
                    "])"
                )
                ui.Text("Horizontal and Vertical create split panes; Tabs overlays Views in one pane.")
    return layout_usage


def build_animated_pixels():
    """Create the pixels used by the RenderApp-owned gallery image."""
    width, height = 320, 180
    x = np.linspace(0, 255, width, dtype=np.uint16)
    y = np.linspace(0, 255, height, dtype=np.uint16)
    xx, yy = np.meshgrid(x, y)

    def animated_pixels(phase):
        return np.stack(
            [
                (xx + phase) % 256,
                (yy + phase * 2) % 256,
                (255 - xx + phase * 3) % 256,
            ],
            axis=2,
        ).astype(np.uint8)

    return animated_pixels


def build_ui_image_views():
    """Build empty Image slots that can join the initial ViewerBlueprint."""
    with ui.View(title="UI Image Gallery") as image_panel:
        with ui.Column():
            ui.Text("Animated ui.Image examples")
            with ui.Section("RenderApp.create_image"):
                show_code("image = render.create_image(pixels)")
                ui.Text("Pixel assets are created and owned by their RenderApp.")
            with ui.Section("Embedded ui.Image"):
                show_code("embedded = ui.Image(width=320)")
                embedded_preview = ui.Image(width=320)
            with ui.Section("Standalone ui.Image"):
                show_code('contained = ui.Image(title="Contained Image Preview")')
                show_code('stretched = ui.Image(title="Stretched Image Preview", keep_aspect_ratio=False)')
                ui.Text("Empty slots can be direct children of the initial Layout.")
            with ui.Section("Bind the slots"):
                show_code("embedded.set_image(image); contained.set_image(image)")
            with ui.Section("Pixel updates"):
                show_code("image.pixels = next_pixels; render.sync(data)")
                ui.Text("All three Images update without rebuilding their Views.")

    contained_preview = ui.Image(
        title="Contained Image Preview",
        fit_to_size=True,
        keep_aspect_ratio=True,
    )
    stretched_preview = ui.Image(
        title="Stretched Image Preview",
        fit_to_size=True,
        keep_aspect_ratio=False,
    )
    image_slots = (embedded_preview, contained_preview, stretched_preview)
    return image_panel, contained_preview, stretched_preview, image_slots


def build_ui_gallery(controller):
    free_camera_view = ui.Viewport(
        camera=ui.CameraRef.system(),
        title="Free Camera",
    )
    gallery, lifecycle, lookup_frame, demo_frame = build_python_views(controller)
    simulation_control = build_simulation_control_view(controller)
    layout_usage = build_layout_usage_view()
    image_panel, contained_preview, stretched_preview, image_slots = build_ui_image_views()

    blueprint = ui.Vertical(
        [
            ui.Horizontal(
                [
                    free_camera_view,
                    simulation_control,
                ]
            ),
            ui.Horizontal(
                [
                    ui.Tabs([gallery, lifecycle]),
                    layout_usage,
                    ui.Options(title="Options"),
                    ui.Tabs([image_panel, contained_preview, stretched_preview]),
                ]
            ),
        ]
    )
    return blueprint, gallery, lookup_frame, demo_frame, image_slots


def main():
    model = load_model("examples/assets/model.xml")
    data = SceneData(model)
    slider_actuator = model.get_actuator("actuator_slider")
    slider_force = 10

    def move_slider():
        nonlocal slider_force
        slider_actuator.set_ctrl(data, slider_force)
        slider_force = -slider_force

    controller = UiGalleryController(move_slider)
    gallery_pixels = build_animated_pixels()
    blueprint, gallery, lookup_frame, demo_frame, image_slots = build_ui_gallery(controller)

    with RenderApp(blueprint=blueprint) as render:
        controller.bind(render, gallery, lookup_frame, demo_frame)
        gallery_image = render.create_image(gallery_pixels(0))
        for image_slot in image_slots:
            image_slot.set_image(gallery_image)
        render.launch(model)
        image_phase = 0

        def render_step():
            nonlocal image_phase
            image_phase = (image_phase + 2) % 256
            gallery_image.pixels = gallery_pixels(image_phase)
            render.sync(data)
            controller.poll_demo_controls()
            if render.input.is_key_just_pressed("f5"):
                controller.hide_selection()
            if render.input.is_key_just_pressed("f6"):
                controller.show_selection()

        run.render_loop(
            model.options.timestep,
            60,
            lambda: step(model, data),
            render_step,
        )


if __name__ == "__main__":
    main()
