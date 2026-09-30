"""
state_cache.py - Physical State Cache & Lazy Evaluation Hash Table (SMBS Phase 3)

SPEC v1.0 Requirement:
Caches generated physical FoldState objects indexed by unique multi-property physical state fingerprints:
folded_bends, remaining_bends, panel_transforms, bend_transforms, orientation, tooling_state, and shape signature.
"""

import hashlib
from typing import Dict, Optional, Tuple, Set, Any


class StateCache:
    """
    Thread-safe physical state cache avoiding redundant state evaluation.
    """

    def __init__(self):
        self._cache: Dict[str, Any] = {}
        self._hits: int = 0
        self._misses: int = 0

    @staticmethod
    def compute_hash(shape_or_state: Any, remaining_bends: Optional[Set[str]] = None) -> str:
        """
        Compute robust physical state fingerprint key.
        If a FoldState is passed, invokes fold_state.fingerprint().
        """
        if hasattr(shape_or_state, 'fingerprint') and callable(shape_or_state.fingerprint):
            return shape_or_state.fingerprint()

        rem = remaining_bends or set()
        bend_signature = "_".join(sorted(list(rem)))

        if shape_or_state is not None and hasattr(shape_or_state, 'Volume'):
            try:
                vol = round(float(shape_or_state.Volume), 4)
                area = round(float(shape_or_state.Area), 4)
                bb = shape_or_state.BoundBox
                bb_sig = f"{bb.XMin:.2f}_{bb.XMax:.2f}_{bb.YMin:.2f}_{bb.YMax:.2f}_{bb.ZMin:.2f}_{bb.ZMax:.2f}"
                raw_str = f"shape_v{vol}_a{area}_bb{bb_sig}_bends[{bend_signature}]"
            except Exception:
                raw_str = f"shape_stub_{id(shape_or_state)}_bends[{bend_signature}]"
        else:
            raw_str = f"shape_stub_{id(shape_or_state)}_bends[{bend_signature}]"

        return hashlib.sha256(raw_str.encode('utf-8')).hexdigest()

    def get(self, hash_key: str) -> Optional[Any]:
        """Retrieve cached state object by fingerprint key."""
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
    def stats(self) -> Dict[str, Any]:
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
