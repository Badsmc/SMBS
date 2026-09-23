import os, sys, json, unittest, py_compile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.abspath(__file__))

print("=== syntax check ===")
for rel in ("core/bend_matcher.py", "export/report_writer.py",
            "export/sequence_writer.py"):
    p = os.path.join(ROOT, rel)
    if not os.path.isfile(p):
        print("  SKIP:", rel); continue
    try:
        py_compile.compile(p, doraise=True)
        print("  {} ... OK".format(rel))
    except py_compile.PyCompileError as e:
        print("  {} ... FAIL: {}".format(rel, e))

print()
print("=== imports ===")
try:
    from core import bend_matcher as bm
    print("  bend_matcher OK, PART_OK:", bm.PART_OK)
except Exception as e:
    print("  bend_matcher FAIL:", e)

try:
    from export import report_writer as rw
    print("  report_writer OK")
except Exception as e:
    print("  report_writer FAIL:", e)

try:
    from export import sequence_writer as sw
    print("  sequence_writer OK")
except Exception as e:
    print("  sequence_writer FAIL:", e)

print()
print("=== unittest discovery ===")
for top in ("core/tests", "autosequencer/tests"):
    path = os.path.join(ROOT, top)
    if not os.path.isdir(path):
        print("  {} MISSING".format(top)); continue
    loader = unittest.TestLoader()
    suite = loader.discover(path, pattern="test_*.py")
    runner = unittest.TextTestRunner(verbosity=0)
    r = runner.run(suite)
    print("  {}: tests={} fail={} err={}".format(
        top, r.testsRun, len(r.failures), len(r.errors)))

print()
print("DONE")
