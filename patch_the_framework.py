from os.path import join, isfile

Import("env")

def patch_ble_store_config_h():
    framework_dir = env.PioPlatform().get_package_dir("framework-espidf")
    file_path = join(
        framework_dir,
        "components",
        "bt",
        "host",
        "nimble",
        "nimble",
        "nimble",
        "host",
        "store",
        "config",
        "include",
        "store",
        "config",
        "ble_store_config.h",
    )

    # Check if the file exists
    assert isfile(file_path), f"File not found: {file_path}"

    # Define the line to search for and the line to add
    search_line = (
        "int ble_store_config_delete(int obj_type, const union ble_store_key *key);"
    )
    add_line = "void ble_store_config_init(void); // added"

    # Initialize a flag to check if the file has been modified
    modified = False

    # Open the file and read its contents
    with open(file_path, "r") as file:
        lines = file.readlines()

    # Check if the line to add is already in the file
    if add_line not in lines:
        # Iterate over the lines to find the search line
        for i, line in enumerate(lines):
            if line.strip() == search_line:
                # Check if the next line is not the line to add
                if i + 1 < len(lines) and lines[i + 1].strip() != add_line:
                    # Insert the line to add just below the search line
                    lines.insert(i + 1, add_line + "\n")
                    modified = True
                    break

    # If the file has been modified, write the changes back to the file
    if modified:
        with open(file_path, "w") as file:
            file.writelines(lines)
        print("Patched ble_store_config.h")


patch_ble_store_config_h()

# --- Replace some file from a build process with another
# this works only with the project files and not with the framework files
# def replace_node_with_another(env, node):
#     return env.File("ble_store_config_patched.h")
#     return node

# env.AddBuildMiddleware(
#     replace_node_with_another,
#     "framework-espidf/components/bt/host/nimble/nimble/nimble/host/store/config/include/store/config/ble_store_config.h"
# )
