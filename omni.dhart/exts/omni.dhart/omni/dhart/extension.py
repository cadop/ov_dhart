import os 
import omni.ext
import omni.ui as ui
from omni.kit.quicklayout import QuickLayout

from . import core

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.


class DhartExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.

    MENU_PATH = "Analysis/DHART"
    _instance = None

    def on_startup(self, ext_id):

        print("[omni.dhart] MyExtension startup")

        # # Add a window menu item so we can easily find the package
        # self._window = None
        # try:
        #     self._menu = omni.kit.ui.get_editor_menu().add_item(
        #         DhartExtension.MENU_PATH, self.show_window, toggle=True, value=False
        #     )
        # except Exception as e:
        #     self._menu = None

        self.initialize()

        ### Subscribe to events
        self._usd_context = omni.usd.get_context()
        self._selection = self._usd_context.get_selection()
        self._events = self._usd_context.get_stage_event_stream()
        self._stage_event_sub = self._events.create_subscription_to_pop(self._on_stage_event, 
                                                                        name='my stage update'
                                                                        )

        self.show_window()

        DhartExtension._instance = self


    def show_window(self):
        print('Passed Class instantiation')
        # if value:
 
        # self._clean_up_window()
        self.initialize()

        self._window = ui.Window("DHART", width=300, height=300)
        # self._window.set_visibility_changed_fn(self._visibility_changed_fn)

        with self._window.frame:
            with ui.VStack():
                # ui.Label("Some Label")
                ui.Button("Set Current Scene", clicked_fn=lambda: self.DI.scene_setup())
                ui.Button("Set Start", clicked_fn=lambda: self.DI.set_as_start())
                ui.Button("Generate BVH", clicked_fn=lambda: self.DI.set_as_mesh())
                ui.Button("Generate Graph", clicked_fn=lambda: self.DI.graph())
                ui.Button("Viz Points", clicked_fn=lambda: self.DI.create_geompoints())

                # with ui.HStack():
                #     ui.Label("timesteps: ")
                #     dt = ui.IntField(height=20)
                #     dt.model.set_value(60)
                #     dt.model.add_value_changed_fn(lambda m: self.Sim.set_sim_steps(m.get_value_as_int()))

        # if self._menu:
        #     print("Setting menu")
        #     omni.kit.ui.get_editor_menu().set_value(DhartExtension.MENU_PATH, True)
        # else:
        #     print("Couldn't setup ")

    def initialize(self):
        ''' Initializer for setting up the class reference to our other functions '''
        self.DI = core.DhartInterface()

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

        print(f'== selection changed with {len(selection)} items')

        prim_selections = [] # Empty list to be filled of selection prims

        # Check if there is a valid selection and stage
        if selection and stage:
            # Make a list of the selection prims
            for selected_path in selection:
                print(f' item {selected_path}:')
                prim = stage.GetPrimAtPath(selected_path)
                prim_selections.append(prim)
                
                # Get the name of this prim, and if its the start, set it 
                # in the other class
                # name = prim.GetName()
                # if name == 'Start':
                    # self.DI.start_prim=prim

                # prim_type = prim.GetTypeName()
                # print(prim_type)
                # if prim_type == 'Mesh':
                #     prim_selections.append(prim)

        if prim_selections:
            core.DhartInterface.active_selection = prim_selections

        # # Call our meshinfo conversion if we have data in selection
        # if prim_selections:
        #     self.DI.scene_setup()
        #     self.DI.set_active_mesh(prim_selections)

    def _visibility_changed_fn(self, value):
        if self._menu:
            omni.kit.ui.get_editor_menu().set_value(self._menu_path, value)

    def on_shutdown(self):
        print("[omni.dhart] MyExtension shutdown")

        # # Remove the item for the menu
        # if self._window:
        #     self._window.set_visibility_changed_fn(None)
            
        try:
            self._window = None 
            self._stage_event_sub = None
            # Set the dhart interface instance to None
            self.DI = None

        except: pass
        
        # self._menu = None

        # Run some extra cleanup for the window
        self._clean_up_window()
        # set this class to none as the last thing we do so it can be newly instantiated later
        DhartExtension._instance = None
        # print('Cleaned up')

    def _clean_up_window(self):
        ''' make sure we cleanup the window '''
        if self._window is not None:
            self._window.destroy()
            self._window = None