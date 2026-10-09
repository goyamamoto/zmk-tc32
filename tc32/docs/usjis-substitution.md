# US-JIS Substitution Specification

## 1. Purpose

US-JIS substitution replaces the keyboard usages that the US physical layout of the keyboard (here the Cidoo V75 Pro) produces with input that a host whose keyboard layout is fixed to Japanese interprets as printed on the US keycaps.

This document specifies what is substituted. For how the module implements it, see [US-JIS architecture](usjis-architecture.md). The table was worked out from the externally observable behavior of the US-key-on-JIS-OS key override of Keyboard Quantizer ([sekigon-gonnoc/vial-qmk](https://github.com/sekigon-gonnoc/vial-qmk), branch `keyboard-quantizer-b`); no code, table or comment of it is used.

## 2. Assumptions and terms

- **Substitution (US-JIS substitution)**: Replacing the usage and Shift reported to the JIS host when an input chord matches C01–C20 in section 4. It is distinct from the JIS Henkan (変換) key (`INT4`), the Muhenkan (無変換) key (`INT5`), and IME kana-kanji conversion.
- **Substituted key**: A key whose input at press time was substituted. Whether the same physical key is substituted depends on Shift and the mode at press time (for example, `2` is non-substituted without Shift and substituted as C02 with Shift). Modifier keys are not substituted.
- **Input chord**: The combination of a key usage on the US physical layout and the modifiers active when that key is pressed. Shift can be either physical Shift (an explicit modifier) or Shift added by that key's binding (an implicit modifier such as `&kp LS(x)`). Shift added by another key's binding is not included. Matching uses only Shift; Ctrl, Alt and GUI are not used for matching and are kept in the output (see section 11 for shortcuts). For example, `&kp PLUS` (`LS(EQUAL)`) matches C10. Shift masked by a baseline behavior such as Mod-Morph is not taken into account (see the limits below).
- **Substitutable key**: A key that appears in an input chord of C01–C20 (`` ` ``, `2`, `6`–`0`, `-`, `=`, `[`, `]`, `\`, `;`, `'`). Depending on the conditions at press time, it is either substituted or not.
- **Output chord**: The combination of the key usage and modifiers reported to the JIS host.
- **Shift used for matching**: Left Shift or Right Shift. Character matching treats both as the same Shift condition.
- **JIS symbol key**: A position on the JIS layout such as `^`, `@`, `[`, `]`, `;`, `:`, and backslash/underscore. The usage IDs are fixed at implementation time against the HID Usage Tables and the target host.
- **Identity**: Changing neither the usage, the modifiers, nor the press/release of an input event. The report equals the baseline.
- **Non-substituted**: Not substituting the usage. Equal to identity while no substituted key is held. While a substituted key is held, the report's modifiers follow the conflict policy in section 6.
- **Press record (active entry)**: The record that holds the substitution result, or non-substitution, for each press of a non-modifier key on the keyboard page until release (sections 6 and 7). The record of a substituted key is called a substituted entry, and the record of a key that is not substituted is called a non-substituted entry.
- **Effective mode**: The mode currently applied. It is distinct from the requested mode while a request is pending (section 7).

### Limits of the implementation

The substituter in `src/usjis.c` implements this spec with the following limits.

- Substitution applies only while a layer in `CONFIG_ZMK_USJIS_LAYER_MASK` is active (0: any layer) and no layer in `CONFIG_ZMK_USJIS_LAYER_EXCLUDE_MASK` is. The V75 Pro excludes `0x0c`, its Mac and Mac Fn layers, which sit over the Win base layer (Fn + S turns Mac on, Fn + A off). In Mac mode the mode can be `ENABLED` but every input is identity.
- Press records are identified by the input usage. Assigning the same usage to two positions is not supported: a second press of a held usage replaces its record (S09 is out of scope).
- The baseline mask of a held Mod-Morph key is not obtained. Mod-Morph on substitutable keys is outside the scope. Releasing a Mod-Morph key (such as Fn+Esc) clears the mask in the HID state after its release report; while a substituted key is held, the substituter sets the mask again before its next report, so no report goes out without it.
- Key toggle (`&kt`) on a substitutable key is not supported. `&kt` decides between press and release by whether its input usage is in the report, but a substituted key reports its output usage: every tap is a press, the output stays pressed on the host, and a mode change stays pending until the keyboard is turned off and on. The default keymap has no `&kt`; ZMK Studio can assign one.
- While a substituted key is held, the implicit modifiers of consumer and system usages (for example `&kp LS(C_VOL_UP)`) are not applied to the keyboard report.
- A clear by an endpoint switch is detected on `zmk_endpoint_changed` by checking whether the output usages of the press records are still pressed.

The target host is Windows with the Japanese 106/109 keyboard layout. macOS does not need substitution; in Mac mode every input is identity.

## 3. Substitution mode

The mode has two values.

| Mode | Behavior |
| --- | --- |
| `DISABLED` | Treats all events as identity |
| `ENABLED` | Substitutes only the keyboard-usage input that matches the substitution table. Everything else is non-substituted |

The firmware provides three operations: `ON`, `OFF` and `TOGGLE`. An `ON` or `OFF` whose resulting mode equals its basis in section 7 (the pending requested mode if there is one, otherwise the effective mode) is a successful no-op: it changes neither the pending request nor the effective mode, and saves nothing. So `ON` while `ENABLED` with an `OFF` pending is not a no-op; it replaces the pending request, and the mode stays `ENABLED` (section 7, step 4). The default is `DISABLED` on first boot and when the settings are missing, the settings version does not match, or a value is invalid.

## 4. Canonical substitution table

The output chords below are logical representations that produce the target character on the JIS host. Rows whose output needs no Shift mask the input Shift. Rows that need it add one Shift on the output side, whether the input Shift is left or right. **The added output Shift is always Left Shift (modifier bit `0x02`); even when the input is Right Shift, Right Shift is not output.** This is an implementation-independent spec: character generation does not need the left/right distinction, and the choice is fixed so that the tests and the substituter share the same expected values.

| ID | Input chord | Target character | Output chord for the JIS host |
| --- | --- | --- | --- |
| C01 | `Shift + GRAVE` | `~` | `Shift + JIS_CARET` |
| C02 | `Shift + 2` | `@` | `JIS_AT` |
| C03 | `Shift + 6` | `^` | `JIS_CARET` |
| C04 | `Shift + 7` | `&` | `Shift + 6` |
| C05 | `Shift + 8` | `*` | `Shift + JIS_COLON` |
| C06 | `Shift + 9` | `(` | `Shift + 8` |
| C07 | `Shift + 0` | `)` | `Shift + 9` |
| C08 | `Shift + MINUS` | `_` | `Shift + JIS_RO` |
| C09 | `EQUAL` | `=` | `Shift + MINUS` |
| C10 | `Shift + EQUAL` | `+` | `Shift + SEMICOLON` |
| C11 | `LEFT_BRACKET` | `[` | `JIS_LEFT_BRACKET` |
| C12 | `Shift + LEFT_BRACKET` | `{` | `Shift + JIS_LEFT_BRACKET` |
| C13 | `RIGHT_BRACKET` | `]` | `JIS_RIGHT_BRACKET` |
| C14 | `Shift + RIGHT_BRACKET` | `}` | `Shift + JIS_RIGHT_BRACKET` |
| C15 | `BACKSLASH` | `\` | `JIS_RO` |
| C16 | `Shift + BACKSLASH` | vertical bar (`U+007C`) | `Shift + JIS_YEN` |
| C17 | `Shift + SEMICOLON` | `:` | `JIS_COLON` |
| C18 | `QUOTE` | `'` | `Shift + 7` |
| C19 | `Shift + QUOTE` | `"` | `Shift + 2` |
| C20 | `GRAVE` | `` ` `` | `Shift + JIS_AT` |

`JIS_RO` is International1 (usage `0x87`) on Keyboard page `0x07`, and `JIS_YEN` is International3 (usage `0x89`). The tests (section 10) check that C08, C15 and C16 produce different combinations of usage and modifiers. C15 expects `U+005C`. Some fonts display it with the yen sign glyph, so judge it by the character code and its use (paths, regular expressions, and so on), not by how it looks on screen. The character code and the glyph on Windows have not been checked separately on hardware.

The other JIS positions used are listed below. This does not mean that ZMK's US key names represent the same characters on the JIS host.

| JIS position | Keyboard usage | ZMK identifier |
| --- | --- | --- |
| `JIS_CARET` | `0x2E` | `EQUAL` |
| `JIS_AT` | `0x2F` | `LEFT_BRACKET` |
| `JIS_LEFT_BRACKET` | `0x30` | `RIGHT_BRACKET` |
| `JIS_RIGHT_BRACKET` | `0x32` | `NON_US_HASH` |
| `JIS_COLON` | `0x34` | `SINGLE_QUOTE` |
| `JIS_RO` | `0x87` | `INTERNATIONAL_1` |
| `JIS_YEN` | `0x89` | `INTERNATIONAL_3` |

The resolver (`src/usjis_resolver.c`) and the tests use these values.

## 5. Inputs that are not substituted

Even in `ENABLED`, the following are not substituted (non-substituted).

- Letter and digit input without Shift.
- Symbol input that does not match the conditions of C01 through C20.
- Ctrl, Alt or GUI alone, and ordinary modifier events.
- Consumer usages and system control usages.
- Internal behaviors: Bluetooth, the connection switch, the LED indicators and ZMK Studio unlock.
- The US-JIS mode operations themselves.

Ctrl, Alt and GUI are not used for matching and are kept in the output, so a chord of them with a substitutable key is matched by Shift alone (section 2). For example, `Ctrl + =` becomes `Ctrl + Shift + MINUS`. For grave accent with Alt, see section 8.

These inputs do not change the usage. When they are pressed while a substituted key is held, the report's modifiers follow the conflict policy in section 6. While no substituted key is held, they are identity.

## 6. Press and release rules

1. Make the substitution decision only once, when the physical key is pressed.
2. Include the output usage, the added modifiers and the masked modifiers in the decision result. When substituting, always include the input's left and right Shift in the masked modifiers, and for rows that need Shift, add Left Shift back through the added modifiers (section 4). Do not keep Shift added by the binding in the output either. Keep Ctrl, Alt and GUI.
3. Store the decision result in a press record.
4. On release of the same physical input, do not re-evaluate the current mode or Shift.
5. Release the output usage stored in the press record. The conflict rules determine the modifiers.
6. Delete the press record after release.

As a result, the Shift state at release does not change what is released. In `RShift down -> 2 down -> RShift up -> 2 up`, the key up of `2` releases `0x2F` (C02), chosen at press (S03). In `2 down -> RShift down -> 2 up -> RShift up`, it releases `0x1F` (non-substituted `2`). No different usage is released and no key is left held down.

The character of a substituted input is fixed at press time and does not change when physical Shift is pressed or released while the key is held (repeat produces the same character). Non-substituted input behaves as in the baseline: Shift pressed after the key press applies to the report as is. So when you press `2` and then Shift, the report becomes `Shift + 2`, and if repeat continues on the JIS host, it produces `"` instead of `@`. Whether pressing a modifier stops or continues repeat depends on the host. This is a known constraint.

### Long press

The substituted usage stays pressed in the HID report while the physical key is held. The firmware must not generate repeat by sending strings or tap events repeatedly. The repeat delay and interval are left to the host settings.

### Overlapping and simultaneous presses

- When up to two different substitutable keys are pressed overlapping and released in reverse order, leave no usage or modifier behind.
- Assigning the same usage to several physical positions is not supported (section 2). A count per usage cannot tell which key was released; S09 gives the required result for an implementation that identifies the input source.
- Because HID modifiers apply to the whole report, keys held at the same time have conflicting modifier requests. Resolve conflicts with the following policy, and allow no stuck key or stuck modifier in any case.

### Modifier conflict policy

When a substituted key that masks Shift (such as C02) and a substituted key that adds Shift (such as C09), or a substituted key and a non-substituted key, are pressed together, a single report cannot express the requests of all keys at once. Non-modifier keys on the keyboard page are always kept in the press records, regardless of the mode or whether they are substituted (the records are also used for the pending check in section 7). In `ENABLED`, while one or more substituted keys are held, the substituter uses those records to resolve the conflict with the following policy. The items below are called conflict rules 1–7.

1. The press order is the order of keycode events (with hold-tap and similar behaviors, it can differ from the physical press order).
2. The request of a non-substituted key is the part of the modifiers reported by the baseline on that key down that belongs to the key itself: the modifiers added by the binding (implicit modifiers such as `&kp LS(x)`) and the modifiers masked by a baseline behavior such as Mod-Morph (the implementation does not obtain the baseline mask; section 2). The substituter neither adds nor masks anything further.
3. The report's modifiers are determined from the physical modifiers and the request of the **most recently pressed key** among the held keys.
4. Report the key down of a new key in the same report as the modifiers that reflect that key's request. If the output usage is already pressed, first send a release report for that usage with the previous modifiers unchanged, then press it (the same as the baseline pre-release). At release, the output usage is released only if no other held key reports it.
5. When the most recently pressed key is released, return the modifiers to the request of the most recently pressed key among the remaining keys, and report this in a single report together with the key up. If no key is held, return to the physical modifiers only. Do not delay the modifier change until the next key event. Restore only Shift requests. Once a Ctrl, Alt or GUI request is removed, do not restore it until that key is released (to avoid it being seen as a standalone press).
6. When the physical modifiers change, recalculate with the same rules. Do not treat Shift added by substitution as physical Shift. Ctrl, Alt or GUI requests removed by conflict rule 5 are not restored by this recalculation either. On a physical modifier key event, send one keyboard report even if the result does not change, as the baseline does. On a consumer/system usage event, do not send a keyboard report if the keyboard report's modifiers do not change (the same as the baseline).
7. When all substituted keys are released, return the modifiers to the baseline processing, taking precedence over conflict rule 5 (the baseline clears implicit modifiers on release; the mask of a held baseline behavior is not restored). In this case too, send the key up and the restored modifiers in a single report. For example, if you hold `&kp LS(A)` and press and release a substituted key, the Shift of `A` is not restored. The same applies to `&kp LS(A)` pressed after the substituted key: in C09 → `LS(A)` → release of C09, the Shift of `A`, the most recently pressed key, is also removed. This is the same constraint as the baseline `hid_listener`. Overlapping presses while no substituted key is held give the same result as the baseline.

The rationale is as follows. The character is determined by the modifiers at key down. The host's key repeat normally applies only to the most recently pressed key (not measured here). Giving priority to the most recently pressed key makes both the first character of each key and the character of the repeating key correct. Because of conflict rules 5 and 7, tapping another key while holding `&uc LG(E)` or similar does not press GUI or other modifiers again.

Two alternatives were considered: giving priority to the first pressed substituted key, and combining additions and masks per modifier bit. Neither is adopted: the former produces the wrong first character for keys pressed later, and the latter cannot guarantee either character when an addition and a mask are requested at the same time.

In `DISABLED`, this policy does not apply, and the baseline modifier processing stays as is (identity). A mode change is deferred until all non-modifier keys are released (section 7), so the two kinds of processing never mix among held keys.

The expected results under this policy are as follows.

| Operation | Expected characters | Intermediate modifiers | Test |
| --- | --- | --- | --- |
| Hold `2` (C02) while holding `Shift`, then press `A` | `@` followed by `A` | While `A` is held, the mask is removed. Releasing `A` returns to the mask | S11 |
| Hold `EQUAL` (C09), then press and release `A` | `=` followed by `a` | While `A` is held, the added Shift is removed. Releasing `A` returns to the added Shift | S12 |
| While holding `EQUAL` (C09), press `Shift` and then `2` (C02) | `=` followed by `@` | The mask of C02, pressed later, takes priority. Releasing C02 returns to the added Shift | S13 |
| Hold `Shift + MINUS` (C08) and release `Shift` first | `_` stays | The added Shift is kept. Repeat also produces `_` | S14 |

While a later key is held, the report's modifiers differ from the request of the key pressed earlier. If the host resumes repeat for that earlier key, or if an application uses the modifiers at key up, the result can differ. This is a known constraint.

## 7. Rules during mode changes

If one or more non-modifier keys on the keyboard page are held, whether substituted or not, a mode change is not applied immediately and becomes pending. This prevents keys pressed in `DISABLED` and keys pressed in `ENABLED` from being mixed. Modifier keys (Shift, Ctrl, Alt, GUI) and consumer/system usages are not substituted in either mode, so they do not count toward the pending condition even while held.

1. Calculate the mode after the request. The basis is the pending requested mode if there is one, otherwise the effective mode. `ON` gives `ENABLED`, `OFF` gives `DISABLED`, and `TOGGLE` gives the inverse of the basis.
2. If a non-modifier key is held, store the requested mode as pending. Otherwise, apply it immediately.
3. If another mode operation arrives while a request is pending, recalculate from the pending requested mode as in step 1 and replace it. So repeated `TOGGLE` inverts the latest request, not the effective mode. Two `TOGGLE`s cancel each other: the request returns to the earlier pending request if there was one. Otherwise it equals the effective mode, and step 4 changes nothing (S16).
4. Apply the pending mode right after the last non-modifier key is released. If the mode to apply equals the effective mode, do nothing (and do not save).
5. Persist only the modes that are actually applied.
6. The release of each key follows the result recorded at its press (substituted or non-substituted). Do not re-decide with the mode at release.

The press and release of a mode key are not sent to the host.

### Endpoint switching and disconnection

ZMK clears the HID report whenever the endpoint changes: a change of transport (USB, Bluetooth or none) or of the Bluetooth profile (`update_current_endpoint` in `endpoints.c`). It sets the report's usages and modifiers to 0, sends the empty report to the endpoint in use before the change (nothing is sent when that was none), then raises `zmk_endpoint_changed`. The internal state of explicit, implicit and masked modifiers remains. On the B1 Pro the connection switch changes the endpoint: BT to cable with USB connected goes from Bluetooth to USB, cable to BT the other way. The empty report to the previous endpoint does not always arrive (a Bluetooth link may already be down).

Physical keys that were held still produce release events after the clear. Press records are therefore handled as follows.

- When a clear occurs, mark all press records at that point as cleared. Cleared records count for the pending check, but not for the modifier calculation (selecting the most recently pressed key) or for "a substituted key is held" in conflict rule 7 (section 6).
- Right after the clear, reset the implicit modifiers set by the substituter to 0, return the mask to the value of the held baseline behaviors, and recalculate the modifiers.
- On release of a cleared record of a substituted key, delete the record and do not send a release report for the output usage to the new endpoint. Recalculate the modifiers and report them if they change.
- On release of a record of a non-substituted key, if no substituted key that is not cleared is held, pass the release to the baseline as is, even if the record is cleared (in `DISABLED`, the baseline always processes it). The baseline clears implicit modifiers on release, so not passing it leaves modifiers behind. If a substituted key that is not cleared is held, the substituter handles the release itself and keeps the modifiers determined by section 6 ([US-JIS architecture](usjis-architecture.md) section 7).
- Record and substitute keys pressed after the clear as usual.
- On a switch without a clear, handle the press records and HID state as they are (the same as the baseline).
- A reboot, including turning off and on, loses the press records and any pending request. Pending requests are not saved, so after boot the mode is the saved mode (section 9).
- The substituter detects a clear on `zmk_endpoint_changed` by checking whether the output usages of the press records are still pressed ([US-JIS architecture](usjis-architecture.md) section 8), so that it does not depend on which changes clear.

## 8. Grave accent and OS mode

For the Windows Japanese layout, the spec is C01 and C20. The following have not been checked on hardware.

- Whether `Alt + GRAVE` is needed as the Zenkaku/Hankaku (全角/半角) toggle.
- Whether C01/C20 require the same output chords with macOS Japanese input.
- Which logical modifier to match after the left Alt/left GUI swap.

There is no per-OS special case for grave accent.

## 9. Settings spec

The mode is stored in Zephyr Settings under the key `usjis/mode` as a two-byte record.

| Field | Type | Value |
| --- | --- | --- |
| `version` | `uint8_t` | 1 |
| `enabled` | `uint8_t` | 0 (`DISABLED`) or 1 (`ENABLED`) |

- Before the settings are loaded, the mode is `DISABLED`.
- A missing record, an invalid length, an invalid value or an unknown version gives `DISABLED`, and the reason is logged.
- Only applied mode changes are saved, 2 seconds after the change (`CONFIG_ZMK_USJIS_SETTINGS_SAVE_DELAY_MS`), so repeated operations give one flash write. A pending request is not saved.
- A failed write is logged; the effective mode and the saved mode then differ until the next successful save.
- Clearing all Bluetooth bonds (Fn+Shift+Esc held for 10 seconds) does not delete this record.

## 10. Automated test spec

### Table-driven tests

For each row C01 through C20, verify the following.

- Rows with Shift match with either Left Shift or Right Shift. Rows without Shift match without Shift.
- The press result has the correct usage, added modifiers and masked modifiers.
- Release uses the result from press time.
- Shift conditions not in the table are non-substituted.

### State transition tests

S tests that use Shift use Right Shift (`0x20`) for physical Shift, to tell it apart from the added Left Shift (`0x02`). Reports are written as `{modifier, [usage]}`.

| ID | Operation sequence | Required result |
| --- | --- | --- |
| S01 | With mode disabled, press and release the input chords of C01–C20 | All events are identity |
| S02 | With mode enabled, `RShift down -> C02 down -> C02 up -> RShift up` | Four reports: `{20,[]}`, `{00,[2F]}`, `{20,[]}`, `{00,[]}`. The C02 key up report restores Right Shift |
| S03 | With mode enabled, `RShift down -> C02 down -> RShift up -> C02 up` | Four reports: `{20,[]}`, `{00,[2F]}`, `{00,[2F]}`, `{00,[]}`. Even if Right Shift is released first, the C02 key up releases `0x2F` |
| S04 | With mode enabled, `C09 down -> mode OFF -> C09 up` | OFF is pending and is applied after C09 is released correctly |
| S05 | With mode enabled, `RShift down -> C02 down -> C08 down -> C08 up -> C02 up -> RShift up` | Modifiers in the order `20`, `00`, `02`, `00`, `20`, `00` (6 reports). `0x2F` and `0x87` are each released once, with nothing left behind |
| S06 | With mode enabled, hold a substituted key | The key down is kept, and no sequence of taps is generated |
| S07 | With mode enabled and with mode disabled, press and release a consumer/system usage | Events and reports are unchanged |
| S08 | With mode enabled and with mode disabled, press and release a mode key | The mode key does not appear in host reports |
| S09 | With mode enabled, for two keys A and B that are both assigned `2`: `A down -> RShift down -> B down -> A up -> B up -> RShift up`, and, releasing B first, `A down -> RShift down -> B down -> B up -> A up -> RShift up` | A is non-substituted `2` (`0x1F`), and B is C02 (`JIS_AT`, `0x2F`). In either order, A up releases only `0x1F`, and B up releases only `0x2F`. Nothing is left behind at the end |
| S10 | Boot with missing, corrupted, or unknown-version settings | `DISABLED` |
| S11 | With mode enabled, `RShift down -> C02 down -> A down -> A up -> C02 up -> RShift up` | `@` followed by `A`. Modifiers in the order `20`, `00`, `20` (A key down), `00` (A key up), `20`, `00`. Nothing left behind |
| S12 | With mode enabled, `C09 down -> A down -> A up -> C09 up` | `=` followed by `a`. Modifiers in the order `02`, `00` (A key down), `02` (A key up), `00`. Nothing left behind |
| S13 | With mode enabled, `C09 down -> RShift down -> C02 down -> C02 up -> RShift up -> C09 up` | `=` followed by `@`. Modifiers in the order `02`, `02`, `00` (C02 key down), `02` (C02 key up), `02`, `00`. Nothing left behind |
| S14 | With mode enabled, `RShift down -> C08 down -> RShift up -> C08 up` | Four reports: `{20,[]}`, `{02,[87]}`, `{02,[87]}`, `{00,[]}`. Even with Right Shift input, the output uses Left Shift, and `Shift + JIS_RO` is kept while C08 is held |
| S15 | With mode disabled, `A down -> mode ON -> RShift down -> 2 down -> 2 up -> A up -> 2 down -> 2 up -> RShift up` | The first `Shift + 2` is not substituted (identity because the mode is `DISABLED`). The release of `A` applies ON, and the second one becomes `@` (C02) |
| S16 | With mode enabled, `C09 down -> TOGGLE -> TOGGLE -> C09 up` | Still `ENABLED` after release (the automated test checks this through the internal API; on hardware, tap C09 after release and confirm that `=` appears). The mode does not change, so the settings are not written |
| S17 | With mode enabled and connected over Bluetooth, connect USB, then `C09 down -> slide switch from the wireless position to the cable position -> RShift down -> A down -> A up -> RShift up -> C09 up` | To the connection in use before the switch, one empty report is sent after the C09 press report (`Shift + 0x2D`), as in the baseline (delivery is not guaranteed). Reports after the switch do not contain the C09 output or the added Shift, and `RShift + A` becomes `{20,[04]}` (uppercase `A`). This requires clear detection (section 7). Operate after USB re-enumeration completes. No press record remains after release |
| S18 | With mode enabled, `&kp LS(A) down -> C09 down -> C09 up -> A up` | The C09 key up report has no Shift, and from then on the state is the same as the baseline. The Shift of `A` is not restored (conflict rule 7) |
| S19 | With mode enabled, `A down -> C09 down -> A up -> consumer key down -> consumer key up -> C09 up` | From the C09 key down report through the report just before the C09 key up, no keyboard report loses the added Shift of C09 |
| S20 | With mode enabled, `C09 down -> &kp LC(X) down -> A down -> A up -> Alt down -> Alt up -> X up -> C09 up` | From the key up of `A` on, and also after `Alt` is pressed and released, the Ctrl of `LC(X)` is not restored (conflict rules 5 and 6). Nothing left behind |

### Where the tests run

`tc32/tests/usjis/run.sh tc32` (with `KB1_REPO` set to a zmk-kb1-upstream checkout, whose prepared west workspace and pinned Docker image it borrows) builds the cases in `tc32/tests/usjis/cases/` for ZMK's `native_sim` test board and runs them with mock key events, as ZMK's own tests are run: the whole ZMK application (keymap, layers, hold-tap, `hid_listener`, endpoints) with this module. A test-only module logs every report sent and the log is compared with the expected one. `tc32/tests/usjis/generate.py` writes the cases; their expected reports are written from this document, not produced by running the firmware.

| Test | Case |
| --- | --- |
| C01–C20 | `usjis-table`: all rows, with Left, Right and both Shifts where a row has Shift, both release orders; chords outside the table; Shift from the binding (`&kp PLUS`); Ctrl kept (`&kp LC(EQUAL)`); then S01 over the same rows |
| S02–S08, S11–S20 | `usjis-scenarios`. S17 uses a test-only behavior that clears the reports and raises `zmk_endpoint_changed` as `endpoints.c` does, since `native_sim` has no USB or Bluetooth endpoint |
| S09 (same usage twice) | Not run: the same usage on two positions is not supported (section 2) |
| S10 (settings at boot) | Only the default at boot without settings (the test board has no settings backend) |
| Layers, Fn+Tab, IME keys | `usjis-layers`: four layers with a Win layer held by `&mo` (include mask 0x04), the mode toggled with Fn+Tab in both modes, substitution in Win mode only, Alt from the IME hold-tap kept on a substituted key, the IME tap not substituted; `usjis-exclude`: the V75 Pro's layers (Win base, Win Fn, Mac, Mac Fn; exclude mask 0x0c), keys held across a change of mode |
| Capacity | `usjis-capacity`, `usjis-capacity-conflict`: a 17th key with 16 records, alone and while a substituted key is held, and a pending mode applied at its release |
| Mode notice, settings | `usjis-hook`: `zmk_usjis_mode_applied()` once per change of the effective mode; `usjis-settings`: one write 2 s after the last change, nothing for a pending or cancelled request, the stored mode at boot |

To judge intermediate reports on hardware, capture USB on the host side (USBPcap or similar on Windows).

## 11. Limits and unverified items

| Item | State |
| --- | --- |
| Same usage on several positions | Not supported. A second press of a held usage replaces its record (S09) |
| Key toggle (`&kt`) on substitutable keys | Not supported (section 2): the output stays pressed on the host |
| Mod-Morph, sticky keys and macros on substitutable keys | The baseline mask of a held Mod-Morph key is not obtained, and matching does not combine it with Shift. For example, `Shift + &gresc` masks Shift in the baseline and outputs `` ` ``, but matching makes it C01. The default keymaps have no such binding |
| Macros | Keycode events from `&macro` go through the substituter like key presses. The result on a JIS host has not been checked |
| Grave accent and OS mode | The items in section 8 have not been checked |
| Shortcuts | Ctrl, Alt and GUI are kept, and matching uses Shift alone, so `Ctrl + =` becomes `Ctrl + Shift + MINUS`. How applications interpret such chords has not been checked |
| Input conditions | Kana input, full-width symbol settings and remote desktop (character forwarding and scan code forwarding) have not been checked |
| Disconnection and wake | Press records across Bluetooth disconnection and reconnection have not been tested on hardware. Turning off (soft off) reboots the keyboard, which loses the press records |
| ZMK Studio | Studio lists `&usjis` as US-JIS with Toggle, On and Off, and can put it on any key. The mode itself is not shown in Studio |
