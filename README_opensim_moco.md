# OpenSim Moco + Python – Setup Guide (Conda, Windows 11)

This project uses **OpenSim with Moco** via the official Conda packages.

All instructions below assume **Windows 11** and a 64‑bit Python installation.

---

## 0. Prerequisites

You’ll need:

- **Git**
- **Anaconda** or **Miniconda** (64‑bit, Python 3.x)

After installing Anaconda/Miniconda, make sure you can run:

```bash
conda --version
```

from an **Anaconda Prompt** or your terminal.

---

## 1. Get the project files

- Clone the repository with Git 

From here on, all commands assume your terminal is open in the project root.

---

## 2. Create a Conda environment for OpenSim + Moco

We’ll standardize on **Python 3.13**, which is supported by current OpenSim Conda packages that include Moco.

From an **Anaconda Prompt** in the project root:

```bash
conda init powershell
```

This will initialize and modify profiles in the shell. Be sure to restart powershell in VSCode after that. Then run

```bash
conda create -n opensim_moco_env
conda activate opensim_moco_env
```

You can change the environment name if you like, but everyone on the team should use the **same Python version** for consistency.

---

## 3. Install OpenSim (with Moco) into the environment

Install the OpenSim Conda package (4.5.x or newer, which includes Moco):

```bash
conda install -c opensim-org opensim
```

For reproducibility, we can pin a specific version (we will not do this for now):

```bash
conda install -c opensim-org opensim=4.5
```

Conda will resolve the compatible build for **Python 3.13**.

---

## 4. Install project-specific Python dependencies

We need to now install the `requirements.txt` to install those packages **inside the same environment**:

```bash
# Example – only if such a file exists in this project
pip install -r requirements.txt
```
---

## 5. Verify that OpenSim + Moco works

From the project root, with `opensim_moco_env` activated:

```bash
python
```

Then in the Python prompt, run:

```python
import opensim as osim

print(osim.GetVersionAndDate())   # should print an OpenSim version like 4.5.x

# Check that Moco is available:
study = osim.MocoStudy()
problem = study.updProblem()
solver = study.initCasADiSolver()

print("Moco is working!")
```

If you see no errors and the version/date prints, your environment is correctly set up.

Type `exit()` or press `Ctrl+Z` then Enter (Windows) to leave the Python prompt.

---

## 6. Running the project code

With the environment activated (`conda activate opensim_moco_env`) and from the project root, you can run the project’s scripts. For example:

```bash
# Example 1: run a standalone script
python scripts/run_example.py

# Example 2: run a module if the project is structured as a package
python -m sim.run_example
```

Check this README or the project’s `scripts/` / `examples/` / `sim/` folders for the actual entry points (e.g., the main simulation script for your model).

---

## 7. Typical workflow for team members

Each team member needs to do this **once**:

1. Install Anaconda / Miniconda
2. Get the project files (clone or download)
3. Create & activate the Conda environment
4. Install OpenSim (`opensim`) into the environment
5. Install any project dependencies
6. Run the verification snippet (Section 5)

Daily workflow:

```bash
# From the project root
conda activate opensim_moco_env
python <your_script_here.py>
```

---

## 8. Troubleshooting

### 8.1. `ImportError` or “DLL load failed” for `opensim`

- Make sure you installed `opensim` *inside* the `opensim_moco_env` environment (not in `base`).
- Confirm the environment is active before running Python:
  ```bash
  conda activate opensim_moco_env
  python -c "import opensim as osim; print(osim.GetVersionAndDate())"
  ```

### 8.2. Wrong Python or OpenSim version

- Check Python version:
  ```bash
  python --version
  ```
  should show `3.13.x`.

- Inside Python, confirm OpenSim version:
  ```python
  import opensim as osim
  print(osim.GetVersionAndDate())
  ```

If you still have issues, capture:

- The exact error message
- The output of `conda list opensim`
- Your Python version (`python --version`)

and share them with the team so the environment can be debugged.
