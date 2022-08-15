import os


def save_colors(filename, recognizable_objects):
    """
    Saves the colors of the objects in play in a file.
    :param filename: the name of the file to save the colors in.
    :param recognizable_objects: a list of the recognizable objects in play.
    """
    file_text = "".join([recognizable_object.colors_string for recognizable_object in recognizable_objects])
    if os.path.exists(filename):
        os.remove(filename)
    with open(filename, 'w') as f:
        f.write(file_text)
        print("Colors Saved")


def load_colors(filename, recognizable_objects):
    """
    Loads the colors for the objects in play from the file.
    :param filename: the name of the file the colors are saved in.
    :param recognizable_objects: a list of the recognizable objects in play.
    """
    if not os.path.exists(filename):
        print("ERROR: colors file does not exist")
        return

    with open(filename, 'r') as f:
        lines = f.readlines()

    num_of_bounds = 4
    for i, recognizable_object in enumerate(recognizable_objects):
        bounds = [lines[i * num_of_bounds + j] for j in range(num_of_bounds)]
        recognizable_object.save_colors(bounds)
    print("Colors Loaded")
