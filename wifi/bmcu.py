import socket
import time
import urllib.error
import urllib.request


class BmcuController:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config_error = config.error
        self.gcode = self.printer.lookup_object("gcode")
        self.reactor = self.printer.get_reactor()
        self.name = self._parse_name(config.get_name())

        self.bmcu_addr = config.get("bmcu_addr")
        self.request_timeout = config.getfloat(
            "request_timeout", default=5.0, minval=0.1
        )
        self.busy_timeout = config.getfloat(
            "busy_timeout", default=60.0, minval=1.0
        )
        self.poll_interval = config.getfloat(
            "poll_interval", default=1.0, minval=0.05
        )
        self.start_settle_time = config.getfloat(
            "start_settle_time", default=0.2, minval=0.0
        )

        self.base_url = self._build_base_url(self.bmcu_addr)

        self.gcode.register_mux_command(
            "BMCU_LOAD",
            "BMCU",
            self.name,
            self.cmd_BMCU_LOAD,
            desc="Load filament with the configured BMCU channel",
        )
        self.gcode.register_mux_command(
            "BMCU_UNLOAD",
            "BMCU",
            self.name,
            self.cmd_BMCU_UNLOAD,
            desc="Unload filament with the configured BMCU channel",
        )

    def _parse_name(self, config_name):
        parts = config_name.split(None, 1)
        if len(parts) != 2 or not parts[1].strip():
            raise self.config_error(
                "Section name must be in the form '[bmcu my_name]'"
            )
        return parts[1].strip()

    def _build_base_url(self, addr):
        addr = addr.strip().rstrip("/")
        if not addr:
            raise self.config_error("[bmcu] bmcu_addr cannot be empty")
        if "://" not in addr:
            addr = "http://" + addr
        return addr

    def _request_text(self, path):
        url = self.base_url + path
        request = urllib.request.Request(url=url, method="GET")
        try:
            with urllib.request.urlopen(
                request, timeout=self.request_timeout
            ) as response:
                body = response.read().decode("utf-8", errors="replace")
        except (urllib.error.URLError, socket.timeout, OSError) as exc:
            raise self.gcode.error(
                "BMCU request failed for %s: %s" % (url, exc)
            )
        return body.strip()

    def _get_channel(self, gcmd):
        channel = gcmd.get_int("CHANNEL", minval=0, maxval=3)
        return channel

    def _wait_for_idle(self):
        deadline = time.monotonic() + self.busy_timeout
        while True:
            state = self._request_text("/state").upper()
            if state == "IDLE":
                return
            if state != "BUSY":
                raise self.gcode.error(
                    "Unexpected BMCU state response: %s" % (state,)
                )
            if time.monotonic() >= deadline:
                raise self.gcode.error(
                    "Timed out waiting for BMCU to become IDLE"
                )
            wake_time = self.reactor.monotonic() + self.poll_interval
            self.reactor.pause(wake_time)

    def _run_action(self, gcmd, path, action_name):
        channel = self._get_channel(gcmd)
        response = self._request_text("%s/%d" % (path, channel)).upper()
        if response == "BUSY":
            raise self.gcode.error("BMCU is busy")
        if response != "OK":
            raise self.gcode.error(
                "BMCU %s failed for channel %d: %s"
                % (action_name, channel, response)
            )
        if self.start_settle_time > 0.0:
            wake_time = self.reactor.monotonic() + self.start_settle_time
            self.reactor.pause(wake_time)
        self._wait_for_idle()
        gcmd.respond_info(
            "BMCU %s finished on channel %d" % (action_name, channel)
        )

    def cmd_BMCU_LOAD(self, gcmd):
        self._run_action(gcmd, "/input", "load")

    def cmd_BMCU_UNLOAD(self, gcmd):
        self._run_action(gcmd, "/output", "unload")


def load_config(config):
    raise config.error(
        "Use named sections such as '[bmcu feeder0]' instead of '[bmcu]'"
    )


def load_config_prefix(config):
    return BmcuController(config)
