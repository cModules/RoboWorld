# Contributing to OpenRoboOLP

Thank you for your interest in making industrial robot programming accessible to everyone!

## How to Contribute

### 1. Report Issues
- Use GitHub Issues with templates:
  - Bug Report
  - Feature Request
  - New Robot/Post-Processor Request

### 2. Add a Post-Processor (Easiest)
The fastest way to contribute:
1. Fork the repository
2. Create `python/orolp/post/<brand>_<controller>.py`
3. Implement `BasePostProcessor` (see `docs/post_dev_guide.md`)
4. Add a test in `python/tests/test_post.py`
5. Submit a Pull Request with sample output

### 3. Core Engine Development
For C++ core contributions:
- Follow Google C++ Style Guide
- Add unit tests in `python/tests/` or `src/core/tests/`
- Ensure pybind11 bindings are updated
- Document API changes in `docs/api/`

### 4. Documentation & Tutorials
- Fix typos, clarify explanations
- Add Jupyter notebook tutorials in `examples/tutorials/`
- Translate README to other languages

## Development Setup

```bash
git clone https://github.com/your-org/openrobolp.git
cd openrobolp
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev,all]"

# Run tests
pytest python/tests/ -v

# Format code
black python/
clang-format -i src/core/*.cpp src/core/*.h
```

## Pull Request Process

1. **Branch**: Create a feature branch (`feat/post-fanuc`, `fix/ik-singularity`)
2. **Commit**: Use conventional commits (`feat:`, `fix:`, `docs:`, `refactor:`)
3. **Test**: Ensure all tests pass (`pytest`)
4. **Document**: Update relevant `.md` files
5. **Review**: Request review from maintainers
6. **Merge**: Squash and merge after approval

## Code of Conduct

- Be respectful and constructive
- Acknowledge contributions publicly
- Prioritize safety in robot programming examples
- Respect intellectual property (do not include proprietary controller firmware)

## Questions?

- Discussions: GitHub Discussions
- Email: maintainers@openrobolp.org
