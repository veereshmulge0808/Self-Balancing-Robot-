import os
from pathlib import Path

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles

class HTTPServer:
    def __init__(self, dashboard_path="dashboard.html", static_dir="frontend/dist"):
        self.dashboard_path = Path(dashboard_path)
        self.static_dir = Path(static_dir)
        self.app = FastAPI()

        if self.static_dir.is_dir():
            assets_dir = self.static_dir / "assets"
            if assets_dir.is_dir():
                self.app.mount("/assets", StaticFiles(directory=str(assets_dir)), name="assets")

            self.app.get("/")(self.serve_index)
            self.app.get("/dashboard")(self.serve_dashboard)
            self.app.get("/{path:path}")(self.serve_static)
        else:
            self.app.get("/")(self.serve_dashboard)

    async def serve_index(self) -> FileResponse:
        return FileResponse(str(self.static_dir / "index.html"))

    async def serve_static(self, path: str) -> FileResponse:
        safe_path = os.path.normpath(path).lstrip("/")
        if safe_path.startswith(".."):
            return await self.serve_index()

        file_path = self.static_dir / safe_path
        if file_path.is_file():
            return FileResponse(str(file_path))

        return await self.serve_index()

    async def serve_dashboard(self) -> HTMLResponse:
        with self.dashboard_path.open("r", encoding="utf-8") as f:
            return HTMLResponse(f.read())

    def run(self):
        uvicorn.run(self.app, host="localhost", port=8000, log_level="warning")
