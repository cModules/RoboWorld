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

# Install the hygiene pre-commit hook (blocks credentials before they land)
python3 tools/check_no_secrets.py --install-hook
```

## Repository Hygiene

This repository is **public** (Apache-2.0). Anything you commit is public
forever — deleting it later does not help, because it stays in git history.
So keep the following out of the repository:

| Do not commit | Put it here instead |
|---|---|
| Private repo host URLs (CNB, internal GitLab, …) | Local `.git/config`, or a placeholder in docs |
| Access tokens / passwords | Environment variables, a secret manager, or `git credential helper` |
| `https://<user>:<token>@<host>/…` URLs | SSH keys, or a credential helper |
| Private keys (`*.pem`, `id_rsa*`, …) | Your `~/.ssh/` — never the repo |
| Local env files (`.env`) | `.env.example` with dummy values only |

Two safeguards are in place:

1. **`.gitignore`** blocks the usual credential filenames, so `git add .`
   will not pick them up by accident.
2. **`tools/check_no_secrets.py`** scans the tree for private hosts, inline
   credentials, private keys, and hardcoded tokens. It runs in CI
   (`repo-hygiene` job) and, once installed, as a local `pre-commit` hook:

   ```bash
   python3 tools/check_no_secrets.py              # scan the working tree
   python3 tools/check_no_secrets.py --staged     # scan staged files only
   ```

   Known-leaked tokens are matched by **SHA-256 fingerprint only** — the
   plaintext is never written into the script, and the tool's own output is
   scrubbed, so running it cannot leak anything further.

If something sensitive does get committed, do not just delete it in a new
commit: rewrite the history (`git filter-repo`) and **rotate the credential
first**.

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
