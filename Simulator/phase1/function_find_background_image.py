import os

def find_background_image():
    current_directory = os.getcwd()
    images_in_folder = [file for file in os.listdir(current_directory) if file.endswith(".png") or file.endswith(".jpg")]
    if "background.png" in images_in_folder:
        return os.path.join(current_directory, "background.png")
    else:
        return None
