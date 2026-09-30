"""
panel_graph.py - Topological Panel Tree & Moving Subtree Resolver for SMBS Phase 1

SPEC v1.0 Requirement:
Represents the topological tree of planar panels connected by bend lines.
Computes fixed vs moving panel subtrees for any given bend operation:
For chain A - B - C - D, folding at bend (B|C) yields:
  fixed_set = {A, B}
  moving_subtree = {C, D}
"""

from typing import Dict, Any, List, Set, Optional, Tuple
from collections import deque


class Panel:
    """Represents a single planar wall/face in a sheet metal part."""

    def __init__(self, panel_id: str, face_indices: Optional[List[int]] = None, normal: Optional[Tuple[float, float, float]] = None):
        self.id = panel_id
        self.face_indices = face_indices or []
        self.normal = normal

    def __repr__(self) -> str:
        return f"<Panel {self.id}>"


class PanelGraph:
    """
    Graph representing topological adjacency of planar panels connected via bend lines.
    """

    def __init__(self):
        self.panels: Dict[str, Panel] = {}
        # Adjacency map: panel_id -> Dict[neighbor_panel_id, bend_id]
        self._adj: Dict[str, Dict[str, str]] = {}
        # Bend map: bend_id -> (parent_panel_id, child_panel_id)
        self._bend_edges: Dict[str, Tuple[str, str]] = {}

    def add_panel(self, panel: Panel) -> None:
        """Register a planar panel in the graph."""
        self.panels[panel.id] = panel
        if panel.id not in self._adj:
            self._adj[panel.id] = {}

    def add_bend_connection(self, bend_id: str, parent_panel_id: str, child_panel_id: str) -> None:
        """Register a bend line connecting a parent panel and a child panel."""
        if parent_panel_id not in self.panels:
            self.add_panel(Panel(parent_panel_id))
        if child_panel_id not in self.panels:
            self.add_panel(Panel(child_panel_id))

        self._adj[parent_panel_id][child_panel_id] = bend_id
        self._adj[child_panel_id][parent_panel_id] = bend_id
        self._bend_edges[bend_id] = (parent_panel_id, child_panel_id)

    def moving_subtree(self, bend_id: str) -> Set[str]:
        """
        Compute the set of panel IDs that move when bend_id is rotated.
        
        Disconnecting bend_id splits the panel tree into two subtrees:
        1. Fixed subtree containing parent_panel_id.
        2. Moving subtree containing child_panel_id.
        """
        if bend_id not in self._bend_edges:
            return set()

        parent_id, child_id = self._bend_edges[bend_id]

        # Traverse graph starting from child_id without crossing bend_id (edge to parent_id)
        moving = set()
        queue = deque([child_id])
        moving.add(child_id)

        while queue:
            curr = queue.popleft()
            for neighbor, edge_bend in self._adj.get(curr, {}).items():
                if edge_bend == bend_id:
                    continue  # Do not cross target bend edge
                if neighbor not in moving:
                    moving.add(neighbor)
                    queue.append(neighbor)

        return moving

    def fixed_set(self, bend_id: str) -> Set[str]:
        """
        Compute the set of panel IDs that remain stationary when bend_id is rotated.
        
        fixed_set = all_panels - moving_subtree(bend_id)
        """
        moving = self.moving_subtree(bend_id)
        return set(self.panels.keys()) - moving

    def __len__(self) -> int:
        return len(self.panels)
