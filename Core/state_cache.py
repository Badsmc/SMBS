"""
state_cache.py - Geometric Hash Table & Lazy Evaluation Cache for TopoShape States

Backward planning relies on lazy evaluation: intermediate shapes are generated only on demand
and cached by their geometric hash. If different unfolding branches lead to the exact same shape
state (graph coalescence), the state cache avoids redundant collision checks and unfold computations.
"""

import hashlib
from typing import Dict, Optional, Tuple, Set, Any


class StateCache:
    """
    Thread-safe / execution-wide geometric state cache.
    Stores and retrieves generated TopoShape states indexed by unique geometric signatures.
    """

    def __init__(self):
        self._cache: Dict[str, Any] = {}
        self._hits: int = 0
        self._misses: int = 0

    @staticmethod
    def compute_hash(shape: Any, remaining_bends: Set[str]) -> str:
        """
        Compute a robust geometric hash for a TopoShape combined with remaining bend IDs.
        
        Uses TopoShape properties:
        - Volume / Surface Area
        - Center of Mass (X, Y, Z)
        - BoundBox (XMin, XMax, YMin, YMax, ZMin, ZMax)
        - Remaining bend IDs bitmask / set representation
        """
        bend_signature = "_".join(sorted(list(remaining_bends)))
        
        if shape is not None and hasattr(shape, 'Volume'):
            try:
                vol = round(float(shape.Volume), 4)
                area = round(float(shape.Area), 4)
                bb = shape.BoundBox
                bb_sig = f"{bb.XMin:.2f}_{bb.XMax:.2f}_{bb.YMin:.2f}_{bb.YMax:.2f}_{bb.ZMin:.2f}_{bb.ZMax:.2f}"
                cm = shape.CenterOfMass
                cm_sig = f"{cm.x:.2f}_{cm.y:.2f}_{cm.z:.2f}"
                raw_str = f"shape_v{vol}_a{area}_bb{bb_sig}_cm{cm_sig}_bends[{bend_signature}]"
            except Exception:
                # Fallback if properties fail or stub shape
                raw_str = f"shape_stub_{id(shape)}_bends[{bend_signature}]"
        else:
            raw_str = f"shape_stub_{id(shape)}_bends[{bend_signature}]"

        return hashlib.sha256(raw_str.encode('utf-8')).hexdigest()

    def get(self, hash_key: str) -> Optional[Any]:
        """Retrieve cached state object by geometric hash key."""
        if hash_key in self._cache:
            self._hits += 1
            return self._cache[hash_key]
        self._misses += 1
        return None

    def put(self, hash_key: str, state_object: Any) -> None:
        """Store state object in cache."""
        self._cache[hash_key] = state_object

    def contains(self, hash_key: str) -> bool:
        """Check if hash key exists in cache."""
        return hash_key in self._cache

    @property
    def stats(self) -> Dict[str, int]:
        """Return cache hit/miss statistics."""
        total = self._hits + self._misses
        hit_ratio = (self._hits / total * 100.0) if total > 0 else 0.0
        return {
            "size": len(self._cache),
            "hits": self._hits,
            "misses": self._misses,
            "hit_ratio_percent": round(hit_ratio, 2)
        }

    def clear(self) -> None:
        """Reset the cache."""
        self._cache.clear()
        self._hits = 0
        self._misses = 0
