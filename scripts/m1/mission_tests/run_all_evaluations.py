import os, subprocess, sys

print('========================================', flush=True)
print(' MISSION PLANNER EVALUATION TEST SUITE  ', flush=True)
print('========================================\n', flush=True)

script_dir = os.path.dirname(os.path.abspath(__file__))
scripts = sorted([s for s in os.listdir(script_dir) if s.startswith('test_') and s.endswith('.py')])

for s in scripts:
    script_path = os.path.join(script_dir, s)
    subprocess.run([sys.executable, script_path])
    print('', flush=True)

print('All tests completed.', flush=True)
