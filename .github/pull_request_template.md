## Summary

<!-- Briefly describe the purpose and context of this Pull Request -->

### Type of Change
- [ ] `feat`: New feature / capability
- [ ] `fix`: Bug fix / defect resolution
- [ ] `docs`: Documentation updates or additions
- [ ] `refactor`: Code change that neither fixes a bug nor adds a feature
- [ ] `perf`: Performance improvement
- [ ] `test`: Adding missing tests or correcting existing tests
- [ ] `build` / `ci`: Changes to build scripts, container images, or CI workflow

---

### Key Changes
<!-- List the key modifications made in bullet points -->
- 
- 
- 

---

### Verification & Testing
<!-- Detail how this change was tested and verified -->
- [ ] Passed automated engineering test suite via `./scripts/build-and-package.sh` (All 240 assertions passing)
- [ ] Verified clean compilation with zero warnings/errors in Unity 6 (`6000.3.x`)
- [ ] Verified MAVLink lockstep communication with PX4 SITL (TCP :4560)
- [ ] Verified telemetry streaming with QGroundControl (UDP :14550)
- [ ] Verified ROS 2 Humble uXRCE-DDS bridge topic integrity (if applicable)

---

### Checklist
- [ ] My code adheres to the project's C# and aerospace coding standards.
- [ ] Commit messages follow the [Conventional Commits](https://www.conventionalcommits.org/) convention.
- [ ] Relevant documentation has been updated in the `docs/` folder or README.
- [ ] No proprietary or GPL/LGPL contaminated third-party code has been added.
