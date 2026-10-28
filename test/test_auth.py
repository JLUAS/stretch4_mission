import pytest

from stretch4_mission.auth import COOKIE, Sessions, check_password, hash_password
from stretch4_mission.mission_ui import make_handler


@pytest.fixture
def sessions(tmp_path):
    f = tmp_path / 'admin_password'
    f.write_text(hash_password('correct horse') + '\n')
    return Sessions(f, max_failures=3, lockout_sec=60.0)


def test_password_hash_checks_only_the_right_password():
    stored = hash_password('correct horse')
    assert check_password('correct horse', stored)
    assert not check_password('wrong', stored)
    assert not check_password('correct horse', 'garbage')
    assert hash_password('correct horse') != stored   # salted


def test_login_gives_a_session_and_logout_ends_it(sessions):
    token = sessions.login('correct horse')
    assert sessions.valid(token) and not sessions.valid('forged') and not sessions.valid(None)
    sessions.logout(token)
    assert not sessions.valid(token)


def test_wrong_passwords_lock_the_login(sessions):
    for _ in range(3):
        with pytest.raises(ValueError):
            sessions.login('wrong')
    with pytest.raises(RuntimeError):
        sessions.login('correct horse')   # locked even with the right one


def test_no_password_file_means_no_admin_login(tmp_path):
    with pytest.raises(RuntimeError, match='set_admin_password'):
        Sessions(tmp_path / 'missing').login('anything')


def request(sessions, ip, method, path, cookie=''):
    """Run the handler's access check for one request; returns the refusal code or None when allowed."""
    handler = object.__new__(make_handler(None, sessions))
    handler.client_address, handler.headers = (ip, 50000), {'Cookie': cookie}
    sent = []
    handler._send = lambda code, body, *a, **k: sent.append(code)
    return None if handler._allowed(method, path) else sent[0]


def test_operator_routes_are_open_on_the_lan_and_the_rest_needs_the_login(sessions):
    lan = '192.168.1.50'
    assert request(sessions, lan, 'GET', '/') is None
    assert request(sessions, lan, 'POST', '/api/operator') is None
    assert request(sessions, lan, 'POST', '/api/configs') == 401
    assert request(sessions, lan, 'POST', '/api/missions/delete') == 401
    assert request(sessions, lan, 'POST', '/api/operator?x=1') == 401   # exact paths only
    token = sessions.login('correct horse')
    assert request(sessions, lan, 'POST', '/api/configs', f'{COOKIE}={token}') is None
    assert request(sessions, lan, 'POST', '/api/configs', f'{COOKIE}=forged') == 401


def test_internet_addresses_are_refused_and_the_robot_itself_is_admin(sessions):
    assert request(sessions, '8.8.8.8', 'GET', '/') == 403
    assert request(sessions, '127.0.0.1', 'POST', '/api/configs') is None   # SSH tunnel
