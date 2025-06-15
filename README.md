# Burning At The High Stakes

### High Stakes Season (2024-2025)

VEX Codebase for 7405K, with influence from 4 years of VEX w/ those from class of '22, '23, '24.

## TODOS:

- [ ] Clean up the codebase (never will happen unfortunately, but trajectory stuff needs to be redone 🙂)
    - restructure and remove flags potentially. replace them with signals?
- [ ] Proper voltage scaling in drivetrain between 5.5w and 11w motors so they don't clash.
- [ ] Low latency logger w/ SD Card logging (only system that doesn't run in our cooperative scheduler)
- [ ] Boomerang Motion --> never tested, just a mess for the path following part which shouldn't & probably doesn't work
- [ ] Pure pursuit --> rewrite to use new cord system & remove archaic fttbtjfk code & make it adaptive lookahead
- [ ] Path following instead of trajectory following so my boomerang motion can become time invariant.
- [ ] Clean up splining --> in terms of structuring (change up how trajectory generation works)
- [ ] Fix up pure pursuit to fit with the new coordinate system
- [ ] MPC
- [ ] MCL