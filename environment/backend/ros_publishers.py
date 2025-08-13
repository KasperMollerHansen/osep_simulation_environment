import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core.prims import XFormPrim

from backend.omni_graphs import OmniGraphs


class ClockPublisher:
    def __init__(self):
        topic_name = "/clock"
        omni_graphs = OmniGraphs()
        omni_graphs.clock_graph(topic_name)
        return

class TfPublisher:
    def __init__(self, topic_prefix, drone_prim_path, default_body_children):
        omni_graphs = OmniGraphs()

        prim_path = drone_prim_path
        body_prim_path = prim_path + "/body"
        base_link_prim = XFormPrim(body_prim_path + "/base_link")
        base_link_prim_path = base_link_prim.prim_path

        body_children = self._remove_default_children(body_prim_path, default_body_children)
        if body_children:
            sensor_prims = self._get_all_children(body_children)
        else:
            sensor_prims = []
        omni_graphs.tf_graph(prim_path, base_link_prim_path, sensor_prims, body_prim_path, topic_prefix)
        return

    @staticmethod
    def _remove_default_children(prim_path, default_children):
        prim = prims_utils.get_prim_at_path(prim_path)
        children = prims_utils.get_prim_children(prim)
        filtered_children = [child for child in children if child.GetName() not in default_children]
        return filtered_children

    @staticmethod
    def _get_all_children(prims: list):
        idx = 0
        children_len = 0
        children = prims
        while len(children) > children_len:
            children_len = len(children)
            for i in range(idx, len(children)):
                children += prims_utils.get_prim_children(children[i])
                idx += 1
        children_path = [str(prim.GetPath()) for prim in children]
        return children_path
