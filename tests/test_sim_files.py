"""The simulator page is served as static ES modules from /sim/ (no build step, no password)."""
import os
import re

SIM = os.path.join(os.path.dirname(__file__), "..", "src", "backend", "static", "sim")


def test_sim_redirects_to_the_folder(client):
    r = client.get("/sim", follow_redirects=False)
    assert r.status_code in (301, 302, 307) and r.headers["location"] == "/sim/"


def test_page_and_modules_are_served(client):
    r = client.get("/sim/")
    assert r.status_code == 200 and "text/html" in r.headers["content-type"]
    assert 'type="importmap"' in r.text and 'src="js/main.js"' in r.text
    assert r.headers["cache-control"] == "no-cache"
    assert "text/css" in client.get("/sim/style.css").headers["content-type"]
    # every module a module imports exists and is served as JavaScript (browsers refuse modules otherwise)
    for f in sorted(os.listdir(os.path.join(SIM, "js"))):
        r = client.get(f"/sim/js/{f}")
        assert r.status_code == 200 and "javascript" in r.headers["content-type"], f
        for dep in re.findall(r"from '\./([\w-]+\.js)'", r.text):
            assert os.path.isfile(os.path.join(SIM, "js", dep)), (f, dep)
    assert client.get("/sim/js/nope.js").status_code == 404
