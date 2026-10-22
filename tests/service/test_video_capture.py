"""Local video exports reject foreign origins, bad containers and unknown keys."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from tornado.testing import AsyncHTTPTestCase

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from droneshow.http_server import make_app
from droneshow.core import ServiceCore, ServiceError


class VideoCaptureTests(AsyncHTTPTestCase):
    def get_app(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        return make_app(SimpleNamespace(output_root=Path(self.directory.name)))

    def test_roundtrip_is_fixed_root_and_opaque_name(self):
        # Handler-level EBML/doctype fixture; real recorder file is decoded separately.
        body = b'\x1a\x45\xdf\xa3\x87webm' + bytes(32)
        response = self.fetch('/api/video-captures', method='POST', body=body,
                              headers={'Content-Type': 'video/webm'})
        self.assertEqual(response.code, 200)
        url = json.loads(response.body)['url']
        self.assertRegex(url, r'^/api/video-captures/[a-f0-9]{24}\.webm$')
        download = self.fetch(url)
        self.assertEqual(download.code, 200)
        self.assertEqual(download.body, body)
        self.assertEqual(download.headers['Content-Type'], 'video/webm')
        self.assertEqual(download.headers['X-Content-Type-Options'], 'nosniff')
        self.assertEqual(len(list(Path(self.directory.name).rglob('*.webm'))), 1)

    def test_reject_bad_media_foreign_origin_and_unknown_key(self):
        response = self.fetch('/api/video-captures', method='POST', body=b'not video')
        self.assertEqual(response.code, 400)
        response = self.fetch('/api/video-captures', method='POST', body=b'\x1a\x45\xdf\xa3webm',
                              headers={'Origin': 'https://foreign.invalid'})
        self.assertEqual(response.code, 403)
        self.assertEqual(self.fetch('/api/video-captures/' + '0'*24 + '.webm').code, 404)
        self.assertFalse(list(Path(self.directory.name).rglob('*.webm')))


class OfflineBoundaryTests(unittest.TestCase):
    def test_preview_cannot_replace_running_physical_theme(self):
        core = ServiceCore.__new__(ServiceCore)
        core.runtime = SimpleNamespace(snapshot=lambda: {'status': 'idle'})
        core.offline_runtime = SimpleNamespace(snapshot=lambda: {'status': 'running'})
        with self.assertRaises(ServiceError) as caught:
            core._perform_cinematic({'num_drones': 1000, 'preview': True})
        self.assertEqual(caught.exception.code, 'E_RUNTIME_BUSY')


if __name__ == '__main__':
    unittest.main()
