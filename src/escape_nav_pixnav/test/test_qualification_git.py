from escape_nav_pixnav import qualification


def test_git_helper_preserves_short_status_leading_column(monkeypatch, tmp_path):
    class Result:
        stdout = " M pixnav_check.py\n?? src/new.py\n"

    monkeypatch.setattr(qualification.subprocess, "run", lambda *args, **kwargs: Result())

    output = qualification._git(tmp_path, "status", "--short")

    assert output.splitlines()[0] == " M pixnav_check.py"
