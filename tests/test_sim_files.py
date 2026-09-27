"""The webapp is served as static ES modules from the root (no build step, no password); /sim, where it used to
be, redirects there. The API, the WebSockets and the docs still come first."""
import os
import re

from conftest import H

SIM = os.path.join(os.path.dirname(__file__), "..", "src", "backend", "static", "sim")


def test_old_sim_address_redirects_to_the_root(client):
    for path in ("/sim", "/sim/", "/sim/index.html"):
        r = client.get(path, follow_redirects=False)
        assert r.status_code in (301, 302, 307) and r.headers["location"] == "/", path


def test_page_and_modules_are_served(client):
    r = client.get("/")
    assert r.status_code == 200 and "text/html" in r.headers["content-type"]
    assert 'type="importmap"' in r.text and 'src="js/main.js"' in r.text
    assert r.headers["cache-control"] == "no-cache"
    assert "text/css" in client.get("/style.css").headers["content-type"]
    # every module a module imports exists and is served as JavaScript (browsers refuse modules otherwise)
    for f in sorted(os.listdir(os.path.join(SIM, "js"))):
        r = client.get(f"/js/{f}")
        assert r.status_code == 200 and "javascript" in r.headers["content-type"], f
        for dep in re.findall(r"from '\./([\w-]+\.js)'", r.text):
            assert os.path.isfile(os.path.join(SIM, "js", dep)), (f, dep)
    assert client.get("/js/nope.js").status_code == 404


def test_the_api_still_comes_first(client):
    assert client.get("/api/health", headers=H).json()["connected"]
    assert client.get("/api/health").status_code == 401          # the password still guards it
    assert client.get("/api/nope", headers=H).status_code == 404
    assert client.get("/docs").status_code == 200 and "swagger" in client.get("/docs").text.lower()
