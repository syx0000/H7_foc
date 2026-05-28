"""Unit tests for the serial protocol parser."""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.parser import parse_line, get_channel_names


def test_parse_current_pi():
    line = "current_pi: 512, -256, 1024, -512, 768, 0, 384"
    frame = parse_line(line)
    assert frame is not None
    assert frame.logid == 40
    assert abs(frame.fields['I_q'] - 0.5) < 0.001
    assert abs(frame.fields['I_d'] - (-0.25)) < 0.001
    assert abs(frame.fields['V_q'] - 1.0) < 0.001
    assert abs(frame.fields['V_d'] - (-0.5)) < 0.001
    assert abs(frame.fields['I_q_ref'] - 0.75) < 0.001
    assert abs(frame.fields['I_d_ref'] - 0.0) < 0.001
    assert abs(frame.fields['I_q_ref_filt'] - 0.375) < 0.001


def test_parse_speed():
    line = "speed: 100, 98, 2500, 100, 0"
    frame = parse_line(line)
    assert frame is not None
    assert frame.logid == 50
    assert frame.fields['vel_ref'] == 100
    assert frame.fields['vel_ref_filt'] == 98
    assert frame.fields['dtheta_mech'] == 2500
    assert frame.fields['dtheta_mech_out'] == 100


def test_parse_position():
    line = "position: 90.000000, 89.500000, 0.500000, 12723"
    frame = parse_line(line)
    assert frame is not None
    assert frame.logid == 100
    assert abs(frame.fields['pos_ref'] - 90.0) < 0.01
    assert abs(frame.fields['pos_out'] - 89.5) < 0.01
    assert abs(frame.fields['pos_error'] - 0.5) < 0.01
    assert frame.fields['mech_offset'] == 12723


def test_parse_angle_elec():
    line = "Angle_elec_360: 1024, 32768, 2048, 4096, 100"
    frame = parse_line(line)
    assert frame is not None
    assert frame.logid == 10
    assert abs(frame.fields['now_mechposition'] - 1.0) < 0.001
    assert frame.fields['theta_elec'] == 32768
    assert abs(frame.fields['real_position_out'] - 2.0) < 0.001


def test_parse_voltage():
    line = "current_get: 2048,-1024"
    frame = parse_line(line)
    assert frame is not None
    assert frame.logid == 30
    assert abs(frame.fields['V_q'] - 2.0) < 0.001
    assert abs(frame.fields['V_d'] - (-1.0)) < 0.001


def test_parse_isr_timing():
    line = "adc_isr_us tot:42/60 read:3/5 enc:8/12 pos:10/15 vel:8/10 cur:13/18"
    frame = parse_line(line)
    assert frame is not None
    assert frame.logid == 110
    assert frame.fields['tot_us'] == 42
    assert frame.fields['tot_us_max'] == 60


def test_parse_unknown_line():
    line = "LT H7 foc start"
    frame = parse_line(line)
    assert frame is None


def test_parse_empty_line():
    frame = parse_line("")
    assert frame is None


def test_get_channel_names():
    channels = get_channel_names(40)
    assert 'I_q' in channels
    assert 'V_q' in channels
    assert len(channels) == 7


if __name__ == "__main__":
    test_parse_current_pi()
    test_parse_speed()
    test_parse_position()
    test_parse_angle_elec()
    test_parse_voltage()
    test_parse_isr_timing()
    test_parse_unknown_line()
    test_parse_empty_line()
    test_get_channel_names()
    print("All tests passed!")
