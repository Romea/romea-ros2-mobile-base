# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from romea_common_bringup import MetaDescription
import importlib


# def base_full_name(type, model=""):
#     if model != "":
#         return type + "_" + model
#     else:
#         return type


class MobileBaseMetaDescription:
    def __init__(self, meta_description_filename):
        self.meta_description = MetaDescription(
            "mobile_base", meta_description_filename
        )

    def get_name(self):
        return self.meta_description.get("name")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    def get_model(self):
        return self.meta_description.get("model", "configuration")

    def get_simulation_initial_xyz(self):
        return self.meta_description.get("initial_xyz", "simulation")

    def get_simulation_initial_rpy(self):
        return self.meta_description.get("initial_rpy", "simulation")


def urdf_description(prefix, mode, meta_description_filename):

    meta_description = MobileBaseMetaDescription(meta_description_filename)

    base_type = meta_description.get_type()
    base_model = meta_description.get_model()
    base_bringup = importlib.import_module(base_type + "_bringup")

    if not base_model:
        return base_bringup.urdf_description(prefix, mode)
    else:
        return base_bringup.urdf_description(prefix, mode, base_model)
