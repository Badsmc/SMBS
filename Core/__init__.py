"""
BendSeq Core Package
Provides backward state space representation, geometric hashing cache, canonical fold states, bend & panel graphs, single kinematic backend, and search orchestration.
"""

from .state_cache import StateCache
from .backward_graph import BendState, BackwardGraph
from .orchestrator import BendSeqOrchestrator
from .bend_graph import BendRecord, BendGraph, IncompleteBendError
from .panel_graph import Panel, PanelGraph
from .bend_transform import BendTransform
from .fold_state import FoldState

__all__ = [
    "StateCache",
    "BendState",
    "BackwardGraph",
    "BendSeqOrchestrator",
    "BendRecord",
    "BendGraph",
    "IncompleteBendError",
    "Panel",
    "PanelGraph",
    "BendTransform",
    "FoldState"
]
