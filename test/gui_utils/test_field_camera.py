from gui_utils.field_camera import DRIVER, ELEVATED, FieldCamera, orient_drive

FIELD_W, FIELD_H = 690.875, 317.0
VIEWPORT_W, VIEWPORT_H = 1200.0, 700.0


def _camera(alliance="blue", preset=DRIVER) -> FieldCamera:
    return FieldCamera(FIELD_W, FIELD_H, alliance, VIEWPORT_W, VIEWPORT_H, preset)


def test_centerline_ground_point_projects_to_horizontal_center():
    cam = _camera("blue")
    px, _, _ = cam.project(300.0, FIELD_H / 2.0, 0.0)
    assert abs(px - VIEWPORT_W / 2.0) < 1e-6


def test_off_center_point_moves_away_from_horizontal_center():
    cam = _camera("blue")
    px_center, _, _ = cam.project(300.0, FIELD_H / 2.0, 0.0)
    px_side, _, _ = cam.project(300.0, FIELD_H, 0.0)
    assert px_side != px_center


def test_farther_ground_points_rise_toward_the_horizon():
    cam = _camera("blue")
    _, py_near, depth_near = cam.project(50.0, FIELD_H / 2.0, 0.0)
    _, py_far, depth_far = cam.project(600.0, FIELD_H / 2.0, 0.0)
    assert depth_far > depth_near
    assert py_far < py_near  # smaller screen y = higher in the (top-left-origin) image


def test_scale_shrinks_with_distance():
    cam = _camera("blue")
    near_scale = cam.scale_at(50.0, FIELD_H / 2.0, 0.0)
    far_scale = cam.scale_at(600.0, FIELD_H / 2.0, 0.0)
    assert 0 < far_scale < near_scale


def test_depth_is_monotonic_down_the_field_from_blue():
    cam = _camera("blue")
    depths = [cam.project(x, FIELD_H / 2.0, 0.0)[2] for x in (0.0, 100.0, 300.0, 600.0, FIELD_W)]
    assert depths == sorted(depths)


def test_blue_and_red_are_mirror_images():
    blue = _camera("blue")
    red = _camera("red")
    x, y = 200.0, 80.0
    bx, by, bdepth = blue.project(x, y, 0.0)
    # Red's wall sits at FIELD_W and faces the opposite way, so the point
    # that mirrors (x, y) across the field's long-axis midpoint gives red
    # the same depth and screen height, but a horizontally mirrored
    # screen x (red is looking back the way blue came from).
    rx, ry, rdepth = red.project(FIELD_W - x, y, 0.0)
    assert abs(bx - (VIEWPORT_W - rx)) < 1e-6
    assert abs(by - ry) < 1e-6
    assert abs(bdepth - rdepth) < 1e-6


def test_all_field_points_stay_in_front_of_the_camera():
    # The whole point of eye-behind-the-wall placement: nothing on the
    # field should ever clip through depth zero for either preset.
    for alliance in ("blue", "red"):
        for preset in (DRIVER, ELEVATED):
            cam = FieldCamera(FIELD_W, FIELD_H, alliance, VIEWPORT_W, VIEWPORT_H, preset)
            for x in (0.0, FIELD_W / 2.0, FIELD_W):
                for y in (0.0, FIELD_H / 2.0, FIELD_H):
                    _, _, depth = cam.project(x, y, 0.0)
                    assert depth > 0


def test_unknown_alliance_rejected():
    import pytest

    with pytest.raises(ValueError):
        FieldCamera(FIELD_W, FIELD_H, "green", VIEWPORT_W, VIEWPORT_H, DRIVER)


def test_orient_drive_up_moves_away_from_each_alliances_own_wall():
    # Blue's wall is at x=0, so "up" (away from the driver) should be +x;
    # red's wall is at x=field width, so "up" for red should be -x.
    assert orient_drive("blue", up=1.0, right=0.0) == (1.0, 0.0)
    assert orient_drive("red", up=1.0, right=0.0) == (-1.0, 0.0)


def test_orient_drive_right_matches_what_the_camera_renders_as_screen_right():
    cam_blue = _camera("blue")
    cam_red = _camera("red")
    center = (FIELD_W / 2.0, FIELD_H / 2.0)

    for alliance, cam in (("blue", cam_blue), ("red", cam_red)):
        vx, vy = orient_drive(alliance, up=0.0, right=1.0)
        near_px, _, _ = cam.project(*center, 0.0)
        nudged = (center[0] + vx, center[1] + vy)
        far_px, _, _ = cam.project(*nudged, 0.0)
        assert far_px > near_px  # moving toward "right" input moves right on screen


def test_orient_drive_is_a_pure_rotation_not_a_rescale():
    import math

    vx, vy = orient_drive("blue", up=0.6, right=0.8)
    assert math.isclose(math.hypot(vx, vy), 1.0, abs_tol=1e-9)
