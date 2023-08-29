import omni.ui as ui


def populate_window(DI, render):
    
    _window = ui.Window("DHART", width=600, height=600)
    with _window.frame:
        with ui.VStack():
            with ui.HStack(height=5):
                ui.Button("Set Start Position", clicked_fn=lambda: DI.set_as_start(), height=5)
                # start_pos = ui.MultiIntField(0,0,0)
                with ui.HStack():
                    x = ui.IntField(height=5) 
                    x.model.add_value_changed_fn(lambda m : DI.modify_start(x=m.get_value_as_int()))
                    y = ui.IntField(height=5)
                    y.model.add_value_changed_fn(lambda m : DI.modify_start(y=m.get_value_as_int()))
                    z = ui.IntField(height=5)
                    z.model.add_value_changed_fn(lambda m : DI.modify_start(z=m.get_value_as_int()))
                    
                    DI.gui_start = [x,y,z] 

            with ui.HStack(height=5):
                ui.Button("Set End Position", clicked_fn=lambda: DI.set_as_end(),height=5)
                # end_pos = ui.MultiIntField(0,0,0)
                with ui.HStack():
                    x = ui.IntField(height=5)
                    x.model.add_value_changed_fn(lambda m : DI.modify_end(x=m.get_value_as_int()))
                    y = ui.IntField(height=5)
                    y.model.add_value_changed_fn(lambda m : DI.modify_end(y=m.get_value_as_int()))
                    z = ui.IntField(height=5)
                    z.model.add_value_changed_fn(lambda m : DI.modify_end(z=m.get_value_as_int()))

                    DI.gui_end = [x,y,z]
            
            with ui.HStack(height=5):
                ui.Label(" Max Nodes: ")
                max_nodes = ui.IntField(height=5)
                max_nodes.model.add_value_changed_fn(lambda m : DI.set_max_nodes(m.get_value_as_int()))
                max_nodes.model.set_value(DI.max_nodes)

            with ui.VStack(height=5):
                with ui.HStack(height=5):

                    ui.Label(" Grid Spacing: ")
                    grid_spacing = ui.FloatField(height=5)
                    grid_spacing.model.add_value_changed_fn(lambda m : DI.set_spacing(m.get_value_as_float()))
                    grid_spacing.model.set_value(DI.grid_spacing[0])
                    
                    ui.Label(" Height Spacing: ")
                    height_space = ui.FloatField(height=5)
                    height_space.model.add_value_changed_fn(lambda m : DI.set_height(m.get_value_as_float()))
                    height_space.model.set_value(DI.height)
                    
                with ui.HStack(height=5):

                    ui.Label(" Upstep: ")
                    upstep = ui.FloatField(height=5)
                    upstep.model.add_value_changed_fn(lambda m : DI.set_upstep(m.get_value_as_float()))
                    upstep.model.set_value(DI.upstep)
                    
                    ui.Label(" Upslope: ")
                    upslope = ui.FloatField(height=5)
                    upslope.model.add_value_changed_fn(lambda m : DI.set_upslope(m.get_value_as_float()))
                    upslope.model.set_value(DI.upslope)

                with ui.HStack(height=5):

                    ui.Label(" Downstep: ")
                    downstep = ui.FloatField(height=5)
                    downstep.model.add_value_changed_fn(lambda m : DI.set_downstep(m.get_value_as_float()))
                    downstep.model.set_value(DI.downstep)

                    ui.Label(" Downslope: ")
                    downslope = ui.FloatField(height=5)
                    downslope.model.add_value_changed_fn(lambda m : DI.set_downslope(m.get_value_as_float()))
                    downslope.model.set_value(DI.downslope)

                with ui.HStack(height=5):

                    ui.Label(" Node Size: ")
                    node_size = ui.FloatField(height=5)
                    node_size.model.add_value_changed_fn(lambda m : DI.set_nodesize(m.get_value_as_float()))
                    node_size.model.set_value(DI.node_size)

                    ui.Label(" Path Size: ")
                    path_size = ui.FloatField(height=5)
                    path_size.model.add_value_changed_fn(lambda m : DI.set_pathsize(m.get_value_as_float()))
                    path_size.model.set_value(DI.path_size)
                    
            with ui.HStack(height=5):
                ui.Button("Set Mesh for BVH", clicked_fn=lambda: DI.set_as_bvh(), height=50)
                ui.Button("Set Mesh for Visibility BVH", clicked_fn=lambda: DI.set_as_vis_bvh(), height=50)
            
            ui.Button("Generate Graph", clicked_fn=lambda: DI.generate_graph(), height=50)
            
            with ui.HStack(height=5):
                ui.Button("Visibility Graph", clicked_fn=lambda: DI.visibility_graph(), height=50)
                ui.Button("Visibility Graph to Points", clicked_fn=lambda: DI.visibility_graph_groups(), height=50)
            with ui.HStack(height=5):
                ui.Button("Find Path", clicked_fn=lambda: DI.get_path(), height=50)
                ui.Button("Find Energy Path", clicked_fn=lambda: DI.get_energy_path(), height=50)
                ui.Button("Find Visibility Path", clicked_fn=lambda: DI.get_visibility_path(), height=50)
                ui.Button("Find VG Path", clicked_fn=lambda: DI.get_vg_path(), height=50)
            
            ui.Button("Set Camera on Path", clicked_fn=lambda: render.assign_camera(), height=30)

    return _window