import omni.ext
import omni.ui as ui

from . import core

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class DhartExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[ov_dhart] DHART is startingup")

        # Setup GUI
        self.show_window()

        # Initialize DHART class and event subscription 
        self.initialize()

    def show_window(self):
        self._window = ui.Window("DHART", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Button("Set Start Position", clicked_fn=lambda: self.DI.set_as_start())
                ui.Button("Set Mesh for BVH", clicked_fn=lambda: self.DI.set_as_bvh())
                ui.Button("Generate Graph", clicked_fn=lambda: self.DI.generate_graph())

    def initialize(self):
        ''' Initialization and any setup needed '''
        self.DI = core.DhartInterface()

        ### Subscribe to events
        self._usd_context = omni.usd.get_context()
        self._selection = self._usd_context.get_selection()
        self._events = self._usd_context.get_stage_event_stream()
        self._stage_event_sub = self._events.create_subscription_to_pop(self._on_stage_event, 
                                                                        name='my stage update'
                                                                        )

    def _on_stage_event(self, event):
        ''' subscription to an event on the stage '''

        # When a selection is changed, call our function
        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            self._on_selection_changed()

    def _on_selection_changed(self):
        ''' where we do stuff on the event that notified us a stage selection changed '''

        # Gets the selected prim paths
        selection = self._selection.get_selected_prim_paths()
        stage = self._usd_context.get_stage()
        
        # Empty list to be filled of selection prims
        prim_selections = []

        # Check if there is a valid selection and stage
        if selection and stage:
            # Make a list of the selection prims
            for selected_path in selection:
                prim = stage.GetPrimAtPath(selected_path)
                prim_selections.append(prim)
                
        if prim_selections:
            core.DhartInterface.active_selection = prim_selections
            print(f'Set DI to {prim_selections}')


    def on_shutdown(self):
        print("[ov_dhart] DHART is shutting down")
