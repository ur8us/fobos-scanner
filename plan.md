# Implementation Plan From suggestions.md

This plan converts `suggestions.md` into implementation tasks. Unchecked items
are not implemented yet.

1. [ ] Replace ad-hoc backend POST body parsing with a small, strict parser for the flat JSON objects used by `/api/*`.
   Acceptance: request handlers no longer use repeated `strstr()` plus `sscanf()` key scans for JSON fields, and malformed JSON returns a clear error.

2. [ ] Harden the HTTP request parser and body handling.
   Acceptance: request line, headers, `Content-Length`, body size limits, partial requests, and unsupported methods are handled explicitly instead of being silently truncated or mis-parsed.

3. [ ] Centralize validation for inbound settings.
   Acceptance: sample rate, bandwidth ratio, converter frequency, direct sampling mode, clock source, gains, FFT size, display bins, and visible range are validated in one path before mutating global scanner state.

4. [ ] Make marker saving safe.
   Acceptance: `/api/markers/save` validates marker syntax, rejects malformed marker/group data, writes through a temporary file, and atomically replaces `markers.ini` only after validation succeeds.

5. [ ] Make SSE line payload allocation exact or dynamically grown.
   Acceptance: `publish_scan_line()` cannot overflow or truncate regardless of `display_bins`, metadata digit count, or magnitude values.

6. [ ] Strengthen frontend SSE lifecycle handling.
   Acceptance: EventSource errors close stale streams, clear stale scanning state when appropriate, reconnect deliberately, and reject malformed `line` events before rendering.

7. [ ] Improve frontend API error reporting.
   Acceptance: `apiFetchJson()` exposes HTTP status and parsed server error JSON/text so failed controls can show useful diagnostics instead of returning only `null`.

8. [ ] Throttle frontend localStorage writes.
   Acceptance: visible view state is saved only when it changes and at a bounded rate, not once per waterfall row.

9. [ ] Tighten `display_bins` synchronization between canvas resize and backend rows.
   Acceptance: resize sends one debounced view/bin update, stale-width SSE rows are ignored or adapted safely, and spectrum/waterfall rendering does not assume a mismatched row width.

10. [ ] Persist visible range intentionally.
    Acceptance: backend config can save and restore `visible_start` and `visible_end` when appropriate, while Start still resets zoom only when the UI explicitly requests that behavior.

11. [ ] Fix `clamp_scan_end_to_hardware_limit()` view mutation.
    Acceptance: clamping the configured end frequency adjusts the visible range only when it falls outside valid bounds, and does not unnecessarily discard an existing zoom window.

12. [ ] Make build and run paths configurable.
    Acceptance: Makefile and run scripts keep current defaults but allow overriding agile source/build/install paths and `LD_LIBRARY_PATH` without editing project files.

13. [ ] Add stronger compiler diagnostics and a lightweight lint/check target.
    Acceptance: build remains clean with additional useful warnings such as `-Wextra` and `-Wformat-security`, with deliberate suppressions or code fixes where needed.

14. [ ] Improve static-file loading errors.
    Acceptance: `load_file()` and HTTP responses distinguish missing files, empty files, unreadable files, and allocation failures.

15. [ ] Document startup behavior and dependency layout.
    Acceptance: README explains autostart/web UI behavior, expected sibling library files, local library lookup, and wrapper script assumptions.

16. [ ] Add focused regression tests or test procedures for the hardened paths.
    Acceptance: there is a repeatable checklist or script coverage for parser errors, invalid controls, SSE reconnect, marker save validation, resize/display-bin sync, and config persistence.
