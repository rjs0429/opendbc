# Avante MD (2012) port

Steers by sending the VSM1 torque request to the MDPS in place of the ESC, and follows a lead by tapping the stock
(non-adaptive) cruise buttons.
openpilot's side of the fork is `mdpilot/` in the openpilot repo.

## Upstream registration points

Everything else is in this directory, `dbc/hyundai_avante_2012.dbc`, `safety/modes/avante_md.h` and
`safety/tests/test_avante_md.py`.

| File | Line |
|---|---|
| `car/values.py` | import and `AVANTE_MD` in `Platform` |
| `car/fingerprints.py` | `HYUNDAI_AVANTE_2012` migration entry |
| `car/torque_data/override.toml` | `AVANTE_MD_2012` |
| `car/tests/routes.py` | `AVANTE_MD_2012` in `non_tested_cars` |
| `car/tests/test_fw_fingerprint.py` | `avante_md` query time and the total |
| `car/car.capnp` | `avanteMd @16` |
| `safety/safety.h` | include and registry entry |

## Safety model number

`avanteMd` takes the retired `toyotaIpas @16` slot; `SAFETY_AVANTE_MD 16U` is defined in `avante_md.h`. Appending a new
number collides as soon as upstream adds a brand, so do not append to `SafetyModel`. Logs recorded before the move
carry 35: replay them through panda safety with `--mode 16`.

## Following upstream

opendbc updates its in-tree ports in the same commit that changes an API; this port does not get that. On each sync,
read what upstream changed in `car/hyundai/`, `car/interfaces.py`, `car/structs.py`, `car/car.capnp`,
`car/fw_query_definitions.py` and `safety/declarations.h`, and make the same change here.

## Tests

| Test | Guards |
|---|---|
| `test_registration.py` | safety number matches between capnp and C, unused upstream, accepted by panda |
| `test_safety_agreement.py` | the port and panda safety agree on gear, doors, seatbelt, parking brake, MDPS faults and SAS state |
| `test_tx_contract.py` | panda safety passes every cruise button frame the controller sends in the follow scenarios, with messages fed every control frame |

## Known issue

At the bus rates measured on the car, panda's RES gate cuts RES after a few taps: it raises its tracked set speed by
`AVANTE_MD_RES_STEP_SPEED` per press while one ECM tap moves the car less. `TestTxContractAtBusRates` in
`test_tx_contract.py` replays a climb at bus rates and asserts that RES frames, and only RES frames, are rejected; it
fails once this is fixed, and should then require no rejections.

Not yet cross-checked between the port and panda: EMS6 cruise lamps, VSM1 normal state, VSM2 driver torque and TCS5
wheel speed.
