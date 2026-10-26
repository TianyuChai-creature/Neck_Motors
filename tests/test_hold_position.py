"""Check the fixed-position window against the installed wheel and a pseudo-terminal."""
import io
import os
from pathlib import Path
import sys
import tempfile
from contextlib import redirect_stdout

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import f32c
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'examples'))
import hold_position
from test_pitch_feedback import emulator


if __name__ == '__main__':
    assert f32c.DEFAULT_HOLD_POSITION == f32c.State(0, 0)
    with tempfile.TemporaryDirectory() as folder, emulator(1200) as (device, frames), redirect_stdout(io.StringIO()):
        result = hold_position.run(device, folder, stop_after=.4)
    assert result['status'] == 'released', result
    assert result['disable_requests_sent'] and result['software_close_completed']
    assert result['target_deg'] == {'yaw': 0, 'pitch': 0}
    for axis, target in ((1, 0), (2, 350)):
        goals = [int.from_bytes(frame[3:5], 'big') for frame in frames
                 if frame[1] == axis and frame[2] == 3]
        assert goals == [target], (axis, goals)
    assert not any(frame[2] in (8, 10) for frame in frames)
    assert [frame[1] for frame in frames if frame[2] == 5][-2:] == [1, 2]
    print('Fixed-position window passed: yaw=0, pitch=0, no zero calibration/save, release cleanup verified')
