#!/usr/bin/env python3
"""Admin login for the web UI: one password per robot, hashed on the robot; sessions live in memory.

Set it on the robot:  ros2 run stretch4_mission set_admin_password
"""

import getpass
import hashlib
import hmac
import os
import secrets
import sys
import threading
import time
from pathlib import Path

PASSWORD_FILE = Path(os.path.expanduser('~/.config/stretch4_mission/admin_password'))
COOKIE = 's4m_session'
SESSION_SEC = 12 * 3600


def hash_password(password, salt=None):
    salt = os.urandom(16) if salt is None else salt
    digest = hashlib.scrypt(password.encode(), salt=salt, n=2 ** 14, r=8, p=1)
    return f'scrypt${salt.hex()}${digest.hex()}'


def check_password(password, stored):
    stored = stored.strip()
    try:
        kind, salt, _ = stored.split('$')
        return kind == 'scrypt' and hmac.compare_digest(hash_password(password, bytes.fromhex(salt)), stored)
    except ValueError:
        return False


class Sessions:
    """Logged-in admins. A UI restart logs everyone out; too many wrong passwords lock the login for a while."""

    def __init__(self, password_file=PASSWORD_FILE, max_failures=5, lockout_sec=60.0):
        self.password_file, self.max_failures, self.lockout_sec = Path(password_file), max_failures, lockout_sec
        self.tokens, self.failures, self.locked_until = {}, 0, 0.0
        self.lock = threading.Lock()

    def login(self, password):
        with self.lock:
            now = time.monotonic()
            if now < self.locked_until:
                raise RuntimeError('Too many wrong passwords; wait a minute and try again')
            try:
                stored = self.password_file.read_text()
            except FileNotFoundError:
                raise RuntimeError('No admin password on this robot yet: run '
                                   '`ros2 run stretch4_mission set_admin_password` on it') from None
            if not check_password(str(password), stored):
                self.failures += 1
                if self.failures >= self.max_failures:
                    self.failures, self.locked_until = 0, now + self.lockout_sec
                raise ValueError('Wrong password')
            self.failures = 0
            self.tokens = {t: end for t, end in self.tokens.items() if end > now}
            token = secrets.token_urlsafe(32)
            self.tokens[token] = now + SESSION_SEC
            return token

    def valid(self, token):
        with self.lock:
            return token is not None and self.tokens.get(token, 0.0) > time.monotonic()

    def logout(self, token):
        with self.lock:
            self.tokens.pop(token, None)


def main():
    password = getpass.getpass('New admin password for this robot: ')
    if len(password) < 8:
        sys.exit('Use at least 8 characters.')
    if getpass.getpass('Same password again: ') != password:
        sys.exit('The passwords do not match; nothing saved.')
    PASSWORD_FILE.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(PASSWORD_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as f:
        f.write(hash_password(password) + '\n')
    os.chmod(PASSWORD_FILE, 0o600)   # O_CREAT keeps the mode of a file that already existed
    print(f'Saved in {PASSWORD_FILE}. Restart the UI to log out admins logged in with the old password.')


if __name__ == '__main__':
    main()
