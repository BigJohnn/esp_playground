"""Home Assistant REST 客户端。

只用 REST 而不是 WebSocket：命令是低频的，REST 更简单也更好排查。
"""
from __future__ import annotations

import logging
from typing import Any

import httpx

from config import CONFIG

_LOG = logging.getLogger("ha")


class HomeAssistant:
    def __init__(self, url: str | None = None, token: str | None = None) -> None:
        self.url = (url or CONFIG.ha_url).rstrip("/")
        self.token = token or CONFIG.ha_token
        self._client = httpx.AsyncClient(
            base_url=self.url,
            headers={"Authorization": f"Bearer {self.token}"},
            timeout=10.0,
        )

    async def close(self) -> None:
        await self._client.aclose()

    async def states(self) -> list[dict[str, Any]]:
        r = await self._client.get("/api/states")
        r.raise_for_status()
        return r.json()

    async def lights(self) -> list[dict[str, Any]]:
        return [s for s in await self.states() if s["entity_id"].startswith("light.")]

    async def call(self, domain: str, service: str, **data: Any) -> Any:
        r = await self._client.post(f"/api/services/{domain}/{service}", json=data)
        r.raise_for_status()
        return r.json()

    async def resolve_light(self) -> str | None:
        """确定要控制哪只灯：优先用配置里的实体，否则取第一个 light.*"""
        if CONFIG.light_entity:
            return CONFIG.light_entity
        lights = await self.lights()
        if not lights:
            _LOG.warning("HA 里一个 light.* 实体都没有 —— 小米集成配好了吗？")
            return None
        if len(lights) > 1:
            _LOG.warning(
                "HA 里有 %d 只灯，默认用 %s。要指定请设 LIGHT_ENTITY。",
                len(lights),
                lights[0]["entity_id"],
            )
        return lights[0]["entity_id"]
