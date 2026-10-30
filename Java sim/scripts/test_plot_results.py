"""Plot contract tests; run with python -m unittest discover -s scripts -p 'test_*.py'."""
from pathlib import Path
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from io import StringIO
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pandas as pd
import numpy as np
from plot_results import generate, validate, SYSTEMS, TYPES, main

CONTROLS = ("patrol_deviation_probability", "patrol_max_deviation_duration_min",
            "encounter_rearm_policy", "charge_threshold_wh")


def fixture():
    rows = []
    for kind in TYPES:
        for system in SYSTEMS:
            for scenario in (("small", "large") if kind == "PATROL_DENSITY" else ("small",)):
                for i in range(2):
                    rows.append(dict(schema_version=1, experiment_id="fixture", design_id=f"{kind}_{system}",
                        point_id=f"{scenario}_{i}", experiment_type=kind, system_type=system,
                        resource_scenario=scenario, runs=3, valid_capture_coefficient_runs=3,
                        mean_capture_coefficient=0.4 + i * 0.1, std_capture_coefficient=0.05,
                        patrol_density_robots_per_m=0.01 + (i * 0.01 if kind != "ATTACK_DENSITY" else 0),
                        attack_probability_per_tick=0.1 + (i * 0.2 if kind == "ATTACK_DENSITY" else 0),
                        duration_min=720, perimeter_length_m=200, perimeter_type="LINEAR", patrol_strategy="AsyncPatrol",
                        patrol_deviation_probability=0.1, patrol_max_deviation_duration_min=10,
                        encounter_rearm_policy="EXIT_EITHER_RANGE", charge_threshold_wh=5.0,
                        attack_model="PoissonAttack", attack_duration_min=10,
                        capture_definition="intercepted_over_resolved", cost_model="example",
                        mean_activity_coefficient=2.0, mean_tco_eur=2000 + i * 1000, std_tco_eur=0.02,
                        energy_price_eur_per_kwh=0.25))
    return pd.DataFrame(rows)


class PlotTests(unittest.TestCase):
    def test_cli_requires_an_existing_complete_batch_status(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = Path(directory) / "experiment_summary.csv"
            fixture().to_csv(summary, index=False)
            status = Path(directory) / "status.txt"
            for value in (None, "RUNNING", "FAILED", ""):
                with self.subTest(status=value):
                    if value is not None:
                        status.write_text(value)
                    with patch("plot_results.generate") as generate_mock, redirect_stderr(StringIO()):
                        with self.assertRaises(SystemExit) as failure:
                            main([str(summary), "--output", str(Path(directory) / "plots")])
                        self.assertEqual(1, failure.exception.code)
                        generate_mock.assert_not_called()
            status.write_text("COMPLETE\n")
            with patch("plot_results.generate", return_value=[]) as generate_mock:
                self.assertEqual(0, main([str(summary), "--output", str(Path(directory) / "plots")]))
                generate_mock.assert_called_once()
                pd.testing.assert_frame_equal(fixture(), generate_mock.call_args.args[0])

    def test_added_comparison_controls_must_be_defined_and_constant(self):
        for column, changed in zip(CONTROLS, (0.2, 11, "AFTER_OTHER_ENCOUNTER", 6.0)):
            for value in (changed, None):
                with self.subTest(column=column, value=value):
                    data = fixture(); data.loc[0, column] = value
                    with self.assertRaisesRegex(ValueError, column):
                        validate(data)

    def test_legacy_summaries_warn_about_each_missing_comparison_control(self):
        for missing in (CONTROLS, ("charge_threshold_wh",)):
            with self.subTest(missing=missing):
                data = fixture().drop(columns=list(missing))
                with self.assertWarnsRegex(UserWarning, "not been fully validated as comparable") as warning:
                    actual = validate(data)
                for column in missing:
                    self.assertIn(column, str(warning.warning))
                pd.testing.assert_frame_equal(data, actual)
        data = fixture().drop(columns=["charge_threshold_wh"])
        data.loc[0, "patrol_deviation_probability"] = 0.2
        with self.assertWarns(UserWarning), self.assertRaisesRegex(ValueError, "patrol_deviation_probability"):
            validate(data)

    def test_plot_arrays_and_tco_axes_match_the_input_statistics(self):
        data = fixture()
        # Distinct SDs catch swapped axes; shuffled rows exercise sorting and alignment.
        data["std_capture_coefficient"] = np.arange(len(data)) * 0.001 + 0.01
        data["std_tco_eur"] = np.arange(len(data)) + 2.0
        data = data.sample(frac=1, random_state=7)
        original = data.copy(deep=True)
        rendered = []

        def capture_figure(figure, path):
            axis = figure.axes[0]
            rendered.append((Path(path).name, axis.get_xlabel(), axis.get_ylabel()))

        with tempfile.TemporaryDirectory() as directory, \
                patch("matplotlib.axes.Axes.errorbar", autospec=True) as errorbar, \
                patch("matplotlib.axes.Axes.legend", autospec=True), \
                patch("matplotlib.figure.Figure.savefig", autospec=True, side_effect=capture_figure) as savefig:
            paths = generate(data, directory)
            self.assertEqual(8, len(paths))
            self.assertEqual(8, savefig.call_count)
            self.assertEqual([], list(Path(directory).glob("*.png")))
        pd.testing.assert_frame_equal(original, data)
        for _, xlabel, ylabel in rendered[:3]:
            self.assertEqual("Patrol density (robots/m)", xlabel)
            self.assertEqual("Mean capture coefficient", ylabel)
        for _, xlabel, ylabel in rendered[3:6]:
            self.assertEqual("Attack probability per one-minute tick", xlabel)
            self.assertEqual("Mean capture coefficient", ylabel)
        self.assertEqual(("figure_7_capture_vs_tco.png", "Mean TCO (EUR)", "Mean capture coefficient"), rendered[6])
        self.assertEqual(("figure_8_tco_vs_capture.png", "Mean capture coefficient", "Mean TCO (EUR)"), rendered[7])
        calls = errorbar.call_args_list
        self.assertEqual(15, len(calls))

        def check(call, x, y, xerr, yerr, label):
            args, kwargs = call
            np.testing.assert_allclose(np.asarray(args[1]), x)
            np.testing.assert_allclose(np.asarray(args[2]), y)
            if xerr is None:
                self.assertNotIn("xerr", kwargs)
            else:
                np.testing.assert_allclose(np.asarray(kwargs["xerr"]), xerr)
            np.testing.assert_allclose(np.asarray(kwargs["yerr"]), yerr)
            self.assertEqual(label, kwargs["label"])

        index = 0
        for system in SYSTEMS:
            for scenario in ("large", "small"):
                curve = data[(data.experiment_type == "PATROL_DENSITY") & (data.system_type == system)
                             & (data.resource_scenario == scenario)].sort_values("patrol_density_robots_per_m")
                check(calls[index], [0.01, 0.02], [0.4, 0.5], None,
                      curve.std_capture_coefficient, scenario)
                index += 1
        for system in SYSTEMS:
            curve = data[(data.experiment_type == "ATTACK_DENSITY") & (data.system_type == system)].sort_values("attack_probability_per_tick")
            check(calls[index], [0.1, 0.3], [0.4, 0.5], None,
                  curve.std_capture_coefficient, system.title())
            index += 1
        for swapped in (False, True):
            for system in SYSTEMS:
                curve = data[(data.experiment_type == "TCO") & (data.system_type == system)].sort_values("mean_tco_eur")
                check(calls[index], [0.4, 0.5] if swapped else [2000, 3000],
                      [2000, 3000] if swapped else [0.4, 0.5],
                      curve.std_capture_coefficient if swapped else curve.std_tco_eur,
                      curve.std_tco_eur if swapped else curve.std_capture_coefficient,
                      system.title())
                index += 1
        self.assertEqual(15, index)

    def test_generates_exactly_eight_stable_figures(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = generate(fixture(), directory)
            self.assertEqual(8, len(paths))
            self.assertEqual(8, len(list(Path(directory).glob("*.png"))))
            self.assertTrue(all(p.stat().st_size > 1000 for p in paths))
            self.assertEqual("figure_8_tco_vs_capture.png", paths[-1].name)

    def test_missing_columns_and_undefined_capture_fail_clearly(self):
        with self.assertRaisesRegex(ValueError, "Missing required columns"):
            validate(fixture().drop(columns=["mean_capture_coefficient"]))
        data = fixture(); data.loc[0, "mean_capture_coefficient"] = float("nan")
        with self.assertRaisesRegex(ValueError, "undefined"):
            validate(data)

    def test_mixed_batches_and_missing_designs_are_rejected(self):
        data = fixture(); data.loc[0, "experiment_id"] = "another_batch"
        with self.assertRaisesRegex(ValueError, "one experiment batch"):
            validate(data)
        with self.assertRaisesRegex(ValueError, "all three system types"):
            validate(fixture().query("system_type != 'HYBRID'"))

    def test_control_changes_and_duplicate_summaries_are_rejected(self):
        data = fixture(); data.loc[0, "duration_min"] += 1
        with self.assertRaisesRegex(ValueError, "controlled"):
            validate(data)
        data = fixture()
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            validate(pd.concat([data, data.iloc[:1]], ignore_index=True))

    def test_missing_input_exits_with_failure(self):
        with self.assertRaises(SystemExit) as failure:
            main(["file_that_does_not_exist.csv"])
        self.assertEqual(1, failure.exception.code)


if __name__ == "__main__":
    unittest.main()
