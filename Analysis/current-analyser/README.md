# Robot Current Analyser

A desktop Python app for exploring robot telemetry CSV files and focusing on current draw fields.

## What it does

- Loads large CSV telemetry files.
- Removes completely empty rows and ignores rows that do not contain usable time/current data.
- Finds candidate columns by keyword, such as `amps`, and lets you confirm which ones should be included.
- Supports a user-selected time window in seconds.
- Calculates:
  - average current per field
  - average current per second
  - sum of the 1-second averages across the selected window
  - peak current
  - total current across all selected fields
- Displays summary numbers plus graphs for:
  - total current
  - total current averaged per second
  - each selected field at sample rate and averaged per second

## Setup

```powershell
python -m pip install -r requirements.txt
```

## Run

```powershell
python current-analyser.py
```

## Notes

- If your CSV has a usable time column, select it from the dropdown.
- If not, the app falls back to `row_index / 50`, assuming the telemetry was recorded at 50 Hz.
- The "1 s Avg Sum" metric is a current-based proxy over time. True energy would also require voltage data.
