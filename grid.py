def world_to_grid(x, y, start_x, start_y, cell_size):
    grid_x = round((x - start_x) / cell_size)
    grid_y = round((y - start_y) / cell_size)

    return (grid_x, grid_y)