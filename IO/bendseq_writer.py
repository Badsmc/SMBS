"""
bendseq_writer.py - Machine-Readable BendSeq Exporter

Writes the inverted forward bending sequence to a machine-readable JSON / YAML output format.
"""

import json
import os
from typing import Dict, Any, List


class BendSeqWriter:
    """
    Exports final sequence results to standard machine format.
    """

    def write_bendseq(
        self,
        filename: str,
        metadata: Dict[str, Any],
        sequence: List[Dict[str, Any]]
    ) -> str:
        """
        Export sequence data to a formatted JSON file.

        :param filename: Target output file path.
        :param metadata: Part and solver metadata.
        :param sequence: List of step dictionaries in forward order.
        :return: Absolute file path written.
        """
        output_data = {
            "format": "BendSeq_v1.0",
            "metadata": metadata,
            "sequence_summary": {
                "total_steps": len(sequence),
                "total_flips": sum(1 for s in sequence if s.get("operator_instructions", {}).get("requires_part_flip_180")),
                "tool_stations_used": len(set(s.get("operator_instructions", {}).get("station", 1) for s in sequence))
            },
            "steps": sequence
        }

        abs_path = os.path.abspath(filename)
        with open(abs_path, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2)

        return abs_path
