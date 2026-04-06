import math
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import pandas as pd
except ImportError as exc:  # pragma: no cover - import guard for local setup
    pd = None
    PANDAS_IMPORT_ERROR = exc
else:
    PANDAS_IMPORT_ERROR = None

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except ImportError as exc:  # pragma: no cover - import guard for local setup
    FigureCanvasTkAgg = None
    Figure = None
    MATPLOTLIB_IMPORT_ERROR = exc
else:
    MATPLOTLIB_IMPORT_ERROR = None


SAMPLE_RATE_HZ = 50.0
FALLBACK_TIME_LABEL = "[Row Index / 50 Hz]"


@dataclass
class AnalysisResult:
    cleaned_rows: int
    window_rows: int
    current_columns: list[str]
    window_df: "pd.DataFrame"
    per_second_df: "pd.DataFrame"
    summary_df: "pd.DataFrame"


class ScrollableCheckFrame(ttk.Frame):
    def __init__(self, master: tk.Widget) -> None:
        super().__init__(master)
        self.canvas = tk.Canvas(self, highlightthickness=0, borderwidth=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner = ttk.Frame(self.canvas)
        self.window_id = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")

        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.scrollbar.grid(row=0, column=1, sticky="ns")

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.inner.bind("<Configure>", self._sync_scroll_region)
        self.canvas.bind("<Configure>", self._sync_inner_width)

    def _sync_scroll_region(self, _event: tk.Event) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _sync_inner_width(self, event: tk.Event) -> None:
        self.canvas.itemconfigure(self.window_id, width=event.width)


class TelemetryAnalyserApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Robot Current Analyser")
        self.geometry("1440x920")
        self.minsize(1100, 760)

        self.csv_path: Path | None = None
        self.raw_df: "pd.DataFrame | None" = None
        self.analysis_result: AnalysisResult | None = None
        self.field_vars: dict[str, tk.BooleanVar] = {}

        self.keyword_var = tk.StringVar(value="amps")
        self.time_column_var = tk.StringVar(value=FALLBACK_TIME_LABEL)
        self.field_detail_var = tk.StringVar()
        self.start_time_var = tk.StringVar()
        self.end_time_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Load a CSV file to begin.")
        self.dataset_var = tk.StringVar(value="No file loaded")

        self._configure_style()
        self._build_layout()

        if PANDAS_IMPORT_ERROR or MATPLOTLIB_IMPORT_ERROR:
            self.after(100, self._show_missing_dependency_message)

    def _configure_style(self) -> None:
        self.configure(bg="#f4f1ea")
        style = ttk.Style(self)
        style.theme_use("clam")

        style.configure(".", font=("Segoe UI", 10))
        style.configure("TFrame", background="#f4f1ea")
        style.configure("Panel.TFrame", background="#fbf9f4")
        style.configure("TLabel", background="#f4f1ea", foreground="#243238")
        style.configure("Title.TLabel", font=("Segoe UI Semibold", 17), foreground="#193441")
        style.configure("Subtitle.TLabel", font=("Segoe UI", 10), foreground="#4b5f66")
        style.configure("TButton", padding=(10, 7))
        style.configure("Header.TLabel", font=("Segoe UI Semibold", 11), foreground="#193441")
        style.configure("Treeview", rowheight=28, font=("Segoe UI", 10))
        style.configure("Treeview.Heading", font=("Segoe UI Semibold", 10))
        style.configure("TLabelframe", background="#fbf9f4", borderwidth=1)
        style.configure("TLabelframe.Label", background="#fbf9f4", foreground="#193441")
        style.map("TButton", background=[("active", "#d7e6ea")])

    def _build_layout(self) -> None:
        root = ttk.Frame(self, padding=16, style="Panel.TFrame")
        root.pack(fill="both", expand=True)
        root.grid_columnconfigure(0, weight=0)
        root.grid_columnconfigure(1, weight=1)
        root.grid_rowconfigure(1, weight=1)

        header = ttk.Frame(root, style="Panel.TFrame")
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        header.grid_columnconfigure(1, weight=1)

        ttk.Label(header, text="Robot Telemetry Current Analyser", style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            header,
            text="Load a CSV, confirm the current fields, choose a time range, and inspect summary metrics plus graphs.",
            style="Subtitle.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(2, 0))
        ttk.Label(header, textvariable=self.dataset_var, style="Subtitle.TLabel").grid(row=0, column=1, sticky="e")
        ttk.Label(header, textvariable=self.status_var, style="Subtitle.TLabel").grid(
            row=1, column=1, sticky="e", pady=(2, 0)
        )

        controls = ttk.Frame(root, padding=14, style="Panel.TFrame")
        controls.grid(row=1, column=0, sticky="nsw", padx=(0, 14))
        controls.grid_columnconfigure(0, weight=1)

        data_group = ttk.LabelFrame(controls, text="Dataset", padding=12)
        data_group.grid(row=0, column=0, sticky="ew")
        data_group.grid_columnconfigure(1, weight=1)
        ttk.Button(data_group, text="Open CSV", command=self.load_csv).grid(row=0, column=0, sticky="w")
        ttk.Label(data_group, text="Column keyword").grid(row=1, column=0, sticky="w", pady=(12, 0))
        ttk.Entry(data_group, textvariable=self.keyword_var).grid(row=2, column=0, sticky="ew", pady=(4, 0))
        ttk.Button(data_group, text="Find Matching Fields", command=self.refresh_candidate_fields).grid(
            row=3, column=0, sticky="ew", pady=(10, 0)
        )

        time_group = ttk.LabelFrame(controls, text="Time Window", padding=12)
        time_group.grid(row=1, column=0, sticky="ew", pady=(14, 0))
        time_group.grid_columnconfigure(0, weight=1)
        ttk.Label(time_group, text="Time column").grid(row=0, column=0, sticky="w")
        self.time_combo = ttk.Combobox(
            time_group, textvariable=self.time_column_var, state="readonly", values=[FALLBACK_TIME_LABEL]
        )
        self.time_combo.grid(row=1, column=0, sticky="ew", pady=(4, 8))
        ttk.Label(time_group, text="Start time (seconds, optional)").grid(row=2, column=0, sticky="w")
        ttk.Entry(time_group, textvariable=self.start_time_var).grid(row=3, column=0, sticky="ew", pady=(4, 8))
        ttk.Label(time_group, text="End time (seconds, optional)").grid(row=4, column=0, sticky="w")
        ttk.Entry(time_group, textvariable=self.end_time_var).grid(row=5, column=0, sticky="ew", pady=(4, 0))
        ttk.Button(time_group, text="Run Analysis", command=self.run_analysis).grid(row=6, column=0, sticky="ew", pady=(12, 0))

        fields_group = ttk.LabelFrame(controls, text="Accepted Current Fields", padding=12)
        fields_group.grid(row=2, column=0, sticky="nsew", pady=(14, 0))
        fields_group.grid_rowconfigure(1, weight=1)
        fields_group.grid_columnconfigure(0, weight=1)
        controls.grid_rowconfigure(2, weight=1)

        quick_actions = ttk.Frame(fields_group)
        quick_actions.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        quick_actions.grid_columnconfigure(0, weight=1)
        ttk.Button(quick_actions, text="Select All", command=lambda: self._set_all_fields(True)).grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Button(quick_actions, text="Clear All", command=lambda: self._set_all_fields(False)).grid(
            row=1, column=0, sticky="ew", pady=(6, 0)
        )

        self.field_frame = ScrollableCheckFrame(fields_group)
        self.field_frame.grid(row=1, column=0, sticky="nsew")

        content = ttk.Notebook(root)
        content.grid(row=1, column=1, sticky="nsew")

        self.summary_tab = ttk.Frame(content, padding=10, style="Panel.TFrame")
        self.totals_tab = ttk.Frame(content, padding=10, style="Panel.TFrame")
        self.field_tab = ttk.Frame(content, padding=10, style="Panel.TFrame")
        content.add(self.summary_tab, text="Summary")
        content.add(self.totals_tab, text="Total Current")
        content.add(self.field_tab, text="Field Detail")

        self._build_summary_tab()
        self._build_totals_tab()
        self._build_field_tab()

    def _build_summary_tab(self) -> None:
        self.summary_tab.grid_columnconfigure(0, weight=1)
        self.summary_tab.grid_rowconfigure(1, weight=1)

        self.summary_info_label = ttk.Label(
            self.summary_tab,
            text="Metrics will appear here after you load a file and run the analysis.",
            style="Subtitle.TLabel",
        )
        self.summary_info_label.grid(row=0, column=0, sticky="w", pady=(0, 8))

        columns = ("field", "avg_amps", "avg_per_second", "sum_per_second", "peak_amps")
        self.summary_tree = ttk.Treeview(self.summary_tab, columns=columns, show="headings", height=18)
        headings = {
            "field": "Field",
            "avg_amps": "Avg Current (A)",
            "avg_per_second": "Avg of 1 s Avg (A)",
            "sum_per_second": "1 s Avg Sum",
            "peak_amps": "Peak Current (A)",
        }
        widths = {"field": 260, "avg_amps": 140, "avg_per_second": 150, "sum_per_second": 120, "peak_amps": 140}
        for key in columns:
            self.summary_tree.heading(key, text=headings[key])
            self.summary_tree.column(key, width=widths[key], anchor="center")

        summary_scroll = ttk.Scrollbar(self.summary_tab, orient="vertical", command=self.summary_tree.yview)
        self.summary_tree.configure(yscrollcommand=summary_scroll.set)
        self.summary_tree.grid(row=1, column=0, sticky="nsew")
        summary_scroll.grid(row=1, column=1, sticky="ns")

    def _build_totals_tab(self) -> None:
        self.totals_tab.grid_columnconfigure(0, weight=1)
        self.totals_tab.grid_rowconfigure(1, weight=1)
        ttk.Label(
            self.totals_tab,
            text="Top: summed current at the original sample rate. Bottom: average total current per second.",
            style="Subtitle.TLabel",
        ).grid(row=0, column=0, sticky="w", pady=(0, 8))

        if Figure is None or FigureCanvasTkAgg is None:
            self.total_figure = None
            self.total_canvas = None
            ttk.Label(
                self.totals_tab,
                text="Install matplotlib to enable charts.",
                style="Subtitle.TLabel",
            ).grid(row=1, column=0, sticky="nw")
            return

        self.total_figure = Figure(figsize=(8, 6), dpi=100, facecolor="#fbf9f4")
        self.total_canvas = FigureCanvasTkAgg(self.total_figure, master=self.totals_tab)
        self.total_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")

    def _build_field_tab(self) -> None:
        self.field_tab.grid_columnconfigure(0, weight=1)
        self.field_tab.grid_rowconfigure(2, weight=1)

        selector = ttk.Frame(self.field_tab, style="Panel.TFrame")
        selector.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        selector.grid_columnconfigure(1, weight=1)
        ttk.Label(selector, text="Field", style="Header.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.field_detail_combo = ttk.Combobox(selector, textvariable=self.field_detail_var, state="readonly")
        self.field_detail_combo.grid(row=0, column=1, sticky="ew")
        self.field_detail_combo.bind("<<ComboboxSelected>>", lambda _event: self.update_field_plot())

        ttk.Label(
            self.field_tab,
            text="Top: current draw for the selected field. Bottom: average current per second.",
            style="Subtitle.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(0, 8))

        if Figure is None or FigureCanvasTkAgg is None:
            self.field_figure = None
            self.field_canvas = None
            ttk.Label(
                self.field_tab,
                text="Install matplotlib to enable charts.",
                style="Subtitle.TLabel",
            ).grid(row=2, column=0, sticky="nw")
            return

        self.field_figure = Figure(figsize=(8, 6), dpi=100, facecolor="#fbf9f4")
        self.field_canvas = FigureCanvasTkAgg(self.field_figure, master=self.field_tab)
        self.field_canvas.get_tk_widget().grid(row=2, column=0, sticky="nsew")

    def _show_missing_dependency_message(self) -> None:
        missing = []
        if PANDAS_IMPORT_ERROR:
            missing.append(f"pandas ({PANDAS_IMPORT_ERROR})")
        if MATPLOTLIB_IMPORT_ERROR:
            missing.append(f"matplotlib ({MATPLOTLIB_IMPORT_ERROR})")
        messagebox.showerror(
            "Missing dependencies",
            "This app needs extra Python packages before it can run:\n\n"
            + "\n".join(missing)
            + "\n\nInstall them with:\npython -m pip install -r requirements.txt",
        )

    def load_csv(self) -> None:
        if PANDAS_IMPORT_ERROR or MATPLOTLIB_IMPORT_ERROR:
            self._show_missing_dependency_message()
            return

        csv_path = filedialog.askopenfilename(
            title="Select telemetry CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not csv_path:
            return

        try:
            dataframe = pd.read_csv(csv_path, low_memory=False)
        except Exception as exc:  # pragma: no cover - UI error path
            messagebox.showerror("CSV load failed", f"Could not read the selected file.\n\n{exc}")
            return

        self.csv_path = Path(csv_path)
        self.raw_df = dataframe
        self.dataset_var.set(f"{self.csv_path.name} | {len(dataframe):,} rows x {len(dataframe.columns):,} cols")
        self.status_var.set("CSV loaded. Review the detected current fields and run the analysis.")

        time_options = [FALLBACK_TIME_LABEL, *self._detect_time_columns(dataframe)]
        self.time_combo.configure(values=time_options)
        self.time_column_var.set(time_options[0])

        self.refresh_candidate_fields()

    def refresh_candidate_fields(self) -> None:
        if self.raw_df is None:
            messagebox.showinfo("Load a CSV first", "Choose a telemetry CSV before searching for current fields.")
            return

        keyword = self.keyword_var.get().strip().lower()
        if not keyword:
            messagebox.showwarning("Missing keyword", "Enter a keyword such as 'amps' to find current columns.")
            return

        matches = [column for column in self.raw_df.columns if keyword in column.lower()]
        self._populate_field_checks(matches)
        self.status_var.set(f"Found {len(matches)} columns matching '{keyword}'.")

    def _populate_field_checks(self, fields: list[str]) -> None:
        for widget in self.field_frame.inner.winfo_children():
            widget.destroy()

        self.field_vars.clear()
        if not fields:
            ttk.Label(
                self.field_frame.inner,
                text="No matching columns were found. Try another keyword or inspect the CSV headers.",
                style="Subtitle.TLabel",
                wraplength=280,
                justify="left",
            ).grid(row=0, column=0, sticky="w")
            return

        for idx, field in enumerate(fields):
            variable = tk.BooleanVar(value=True)
            self.field_vars[field] = variable
            ttk.Checkbutton(self.field_frame.inner, text=field, variable=variable).grid(
                row=idx, column=0, sticky="w", pady=2
            )

    def _set_all_fields(self, value: bool) -> None:
        for variable in self.field_vars.values():
            variable.set(value)

    def run_analysis(self) -> None:
        if self.raw_df is None:
            messagebox.showinfo("Load a CSV first", "Choose a telemetry CSV before running the analysis.")
            return

        selected_fields = [field for field, variable in self.field_vars.items() if variable.get()]
        if not selected_fields:
            messagebox.showwarning("No fields selected", "Pick at least one current field to analyse.")
            return

        try:
            result = self._analyse_dataframe(self.raw_df.copy(), selected_fields)
        except ValueError as exc:
            messagebox.showwarning("Analysis input issue", str(exc))
            return
        except Exception as exc:  # pragma: no cover - UI error path
            messagebox.showerror("Analysis failed", f"Something went wrong while analysing the data.\n\n{exc}")
            return

        self.analysis_result = result
        self._update_summary_table()
        self._update_total_plot()
        self._update_field_selector()

        window_min = result.window_df["time_s"].min()
        window_max = result.window_df["time_s"].max()
        self.summary_info_label.configure(
            text=(
                f"Rows after cleanup: {result.cleaned_rows:,} | "
                f"Rows in selected window: {result.window_rows:,} | "
                f"Window: {window_min:.2f}s to {window_max:.2f}s"
            )
        )
        self.status_var.set("Analysis complete.")

    def _analyse_dataframe(self, dataframe: "pd.DataFrame", selected_fields: list[str]) -> AnalysisResult:
        dataframe = dataframe.dropna(how="all").reset_index(drop=True)
        if dataframe.empty:
            raise ValueError("The CSV does not contain any non-empty rows.")

        prepared = pd.DataFrame()
        prepared["time_s"] = self._extract_time_seconds(dataframe)

        for field in selected_fields:
            prepared[field] = pd.to_numeric(dataframe[field], errors="coerce")

        prepared = prepared.dropna(how="all", subset=selected_fields)
        prepared = prepared.dropna(subset=["time_s"]).sort_values("time_s").reset_index(drop=True)
        if prepared.empty:
            raise ValueError("No usable rows remained after removing null telemetry rows.")
        cleaned_rows = len(prepared)

        start_time, end_time = self._parse_time_window(prepared)
        window_df = prepared.loc[prepared["time_s"].between(start_time, end_time, inclusive="both")].copy()
        if window_df.empty:
            raise ValueError("The selected start and end times produced an empty analysis window.")

        second_bucket = window_df["time_s"].apply(math.floor)
        per_second_df = (
            window_df.assign(second_bucket=second_bucket)
            .groupby("second_bucket", sort=True)[selected_fields]
            .mean()
            .reset_index()
            .rename(columns={"second_bucket": "time_s"})
        )

        summary_rows = []
        for field in selected_fields:
            series = window_df[field].dropna()
            per_second_series = per_second_df[field].dropna()
            summary_rows.append(
                {
                    "field": field,
                    "avg_amps": series.mean() if not series.empty else float("nan"),
                    "avg_per_second": per_second_series.mean() if not per_second_series.empty else float("nan"),
                    "sum_per_second": per_second_series.sum() if not per_second_series.empty else float("nan"),
                    "peak_amps": series.max() if not series.empty else float("nan"),
                }
            )

        summary_df = pd.DataFrame(summary_rows)

        window_df["total_current"] = window_df[selected_fields].fillna(0).sum(axis=1)
        per_second_df["total_current"] = per_second_df[selected_fields].fillna(0).sum(axis=1)

        total_row = pd.DataFrame(
            [
                {
                    "field": "TOTAL",
                    "avg_amps": window_df["total_current"].mean(),
                    "avg_per_second": per_second_df["total_current"].mean(),
                    "sum_per_second": per_second_df["total_current"].sum(),
                    "peak_amps": window_df["total_current"].max(),
                }
            ]
        )
        summary_df = pd.concat([summary_df, total_row], ignore_index=True)

        return AnalysisResult(
            cleaned_rows=cleaned_rows,
            window_rows=len(window_df),
            current_columns=selected_fields,
            window_df=window_df,
            per_second_df=per_second_df,
            summary_df=summary_df,
        )

    def _extract_time_seconds(self, dataframe: "pd.DataFrame") -> "pd.Series":
        selected_time_column = self.time_column_var.get()
        if selected_time_column == FALLBACK_TIME_LABEL:
            return pd.Series(dataframe.index / SAMPLE_RATE_HZ, name="time_s")

        raw_time = dataframe[selected_time_column]
        numeric_time = pd.to_numeric(raw_time, errors="coerce")
        if numeric_time.notna().sum() >= max(3, len(raw_time) // 10):
            return numeric_time

        parsed_time = pd.to_datetime(raw_time, errors="coerce")
        if parsed_time.notna().any():
            first_valid = parsed_time.dropna().iloc[0]
            return (parsed_time - first_valid).dt.total_seconds()

        raise ValueError(
            f"Could not interpret the selected time column '{selected_time_column}' as numeric seconds or timestamps."
        )

    def _parse_time_window(self, prepared: "pd.DataFrame") -> tuple[float, float]:
        min_time = float(prepared["time_s"].min())
        max_time = float(prepared["time_s"].max())

        start_text = self.start_time_var.get().strip()
        end_text = self.end_time_var.get().strip()

        start_time = float(start_text) if start_text else min_time
        end_time = float(end_text) if end_text else max_time

        if start_time > end_time:
            raise ValueError("Start time must be less than or equal to end time.")

        return start_time, end_time

    def _detect_time_columns(self, dataframe: "pd.DataFrame") -> list[str]:
        keywords = ("time", "timestamp", "elapsed", "seconds", "sec")
        matches = [column for column in dataframe.columns if any(key in column.lower() for key in keywords)]
        return matches[:25]

    def _update_summary_table(self) -> None:
        if self.analysis_result is None:
            return

        for row_id in self.summary_tree.get_children():
            self.summary_tree.delete(row_id)

        for _, row in self.analysis_result.summary_df.iterrows():
            self.summary_tree.insert(
                "",
                "end",
                values=(
                    row["field"],
                    self._fmt(row["avg_amps"]),
                    self._fmt(row["avg_per_second"]),
                    self._fmt(row["sum_per_second"]),
                    self._fmt(row["peak_amps"]),
                ),
            )

    def _update_total_plot(self) -> None:
        if self.analysis_result is None or self.total_figure is None or self.total_canvas is None:
            return

        self.total_figure.clear()
        raw_ax = self.total_figure.add_subplot(211)
        per_second_ax = self.total_figure.add_subplot(212)

        window_df = self.analysis_result.window_df
        per_second_df = self.analysis_result.per_second_df

        raw_ax.plot(window_df["time_s"], window_df["total_current"], color="#1b6b7a", linewidth=1.2)
        raw_ax.set_title("Total Current at Sample Rate", loc="left", fontsize=12)
        raw_ax.set_ylabel("Amps")
        raw_ax.grid(alpha=0.25)

        per_second_ax.plot(per_second_df["time_s"], per_second_df["total_current"], color="#c96d2d", linewidth=1.8)
        per_second_ax.set_title("Average Total Current per Second", loc="left", fontsize=12)
        per_second_ax.set_xlabel("Time (s)")
        per_second_ax.set_ylabel("Amps")
        per_second_ax.grid(alpha=0.25)

        self.total_figure.tight_layout()
        self.total_canvas.draw()

    def _update_field_selector(self) -> None:
        if self.analysis_result is None:
            return

        fields = self.analysis_result.current_columns
        self.field_detail_combo.configure(values=fields)
        if fields:
            if self.field_detail_var.get() not in fields:
                self.field_detail_var.set(fields[0])
            self.update_field_plot()

    def update_field_plot(self) -> None:
        if self.analysis_result is None or self.field_figure is None or self.field_canvas is None:
            return

        field_name = self.field_detail_var.get()
        if not field_name:
            return

        self.field_figure.clear()
        raw_ax = self.field_figure.add_subplot(211)
        per_second_ax = self.field_figure.add_subplot(212)

        window_df = self.analysis_result.window_df
        per_second_df = self.analysis_result.per_second_df

        raw_ax.plot(window_df["time_s"], window_df[field_name], color="#557a46", linewidth=1.0)
        raw_ax.set_title(f"{field_name} at Sample Rate", loc="left", fontsize=12)
        raw_ax.set_ylabel("Amps")
        raw_ax.grid(alpha=0.25)

        per_second_ax.plot(per_second_df["time_s"], per_second_df[field_name], color="#8b4b65", linewidth=1.8)
        per_second_ax.set_title(f"{field_name} Average per Second", loc="left", fontsize=12)
        per_second_ax.set_xlabel("Time (s)")
        per_second_ax.set_ylabel("Amps")
        per_second_ax.grid(alpha=0.25)

        self.field_figure.tight_layout()
        self.field_canvas.draw()

    @staticmethod
    def _fmt(value: float) -> str:
        return f"{value:,.2f}" if pd.notna(value) else "-"


def main() -> None:
    app = TelemetryAnalyserApp()
    app.mainloop()


if __name__ == "__main__":
    main()
