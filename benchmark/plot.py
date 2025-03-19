#! /usr/bin/env python3
# Plot benchmark results

import json
import typing
import argparse
import itertools
from pathlib import Path

import numpy as np

from bokeh.layouts import column
from bokeh.models import ColumnDataSource, Whisker
from bokeh.plotting import figure, show
from bokeh.transform import factor_cmap, jitter

TimeNS = typing.NewType("TimeNS", int)


class Variance:
    """Variance informations"""

    def __init__(
        self,
        mean: TimeNS,
        median: TimeNS,
        stddev: TimeNS,
    ):
        self.mean = mean
        self.median = median
        self.stddev = stddev

    @staticmethod
    def default():
        return Variance(TimeNS(0), TimeNS(0), TimeNS(0))

    def __repr__(self):
        return f"<Variance {self.mean=}, {self.median=}, {self.stddev=}>"


class Data:
    """Benchmarks data"""

    def __init__(self, name: str, times: list[TimeNS], variance: None | Variance):
        self.name = name
        self.times = times
        self.variance = variance

    @staticmethod
    def from_name(name: str):
        return Data(name, [], None)

    def __repr__(self):
        return f"<Data {self.name=}, {self.times=}, {self.variance=}>"


def make_time_ns(bench: dict) -> TimeNS:
    """Convert JSON output time into TimeNS"""
    t = float(bench["real_time"])
    if bench["time_unit"] == "ns":
        return TimeNS(int(t))
    elif bench["time_unit"] == "us":
        return TimeNS(int(t * 1e3))
    elif bench["time_unit"] == "ms":
        return TimeNS(int(t * 1e6))
    elif bench["time_unit"] == "s":
        return TimeNS(int(t * 1e9))


def parse_json_output(json_content: dict) -> list[Data]:
    """Parse google benchmark JSON output"""
    data_dict = {}
    variance_dict = {}
    for bench in json_content["benchmarks"]:
        run_name = bench["run_name"]
        run_type = bench["run_type"]
        if run_type == "aggregate":
            aggregate_name = bench["aggregate_name"]
            variance = variance_dict.setdefault(run_name, Variance.default())
            if aggregate_name == "mean":
                variance.mean = make_time_ns(bench)
            elif aggregate_name == "median":
                variance.median = make_time_ns(bench)
            elif aggregate_name == "stddev":
                variance.stddev = make_time_ns(bench)
        elif run_type == "iteration":
            data = data_dict.setdefault(run_name, Data.from_name(run_name))
            data.times.append(make_time_ns(bench))

    for k, v in variance_dict.items():
        data_dict[k].variance = v

    return list(data_dict.values())


def class_name(s: str) -> str:
    """Convert data name into a nice label"""
    return s.split("/")[1]


def compute_data_lower_upper(data: Data) -> tuple[float, float]:
    """Compute data lower and upper bounds to display Whisker box"""
    if data.variance:
        v = data.variance
        return (float(v.median - v.stddev), float(v.median + v.stddev))
    else:
        return (float("nan"), float("nan"))


def flatten_data(data: Data) -> list[tuple[str, TimeNS]]:
    """Return a list with data name and time"""
    return [(class_name(data.name), t) for t in data.times]


def plot_batch(datas: list[Data]):
    # Extract all batch classes
    classes = [class_name(d.name) for d in datas]

    # Compute lower and upper bounds
    lower_upper = np.array([compute_data_lower_upper(d) for d in datas])
    lower = lower_upper[:, 0]
    upper = lower_upper[:, 1]

    p = figure(
        height=768,
        sizing_mode="stretch_width",
        x_range=classes,
        background_fill_color="#efefef",
        title="Benchmark results",
    )
    p.xgrid.grid_line_color = None

    # Create whisker plot
    whisker_source = ColumnDataSource(data=dict(base=classes, upper=upper, lower=lower))
    error = Whisker(
        base="base",
        upper="upper",
        lower="lower",
        source=whisker_source,
        level="annotation",
        line_width=2,
    )
    error.upper_head.size = 20
    error.lower_head.size = 20
    p.add_layout(error)

    # Create scatter plot
    flat_data = np.array(
        list(itertools.chain.from_iterable([flatten_data(d) for d in datas]))
    )
    # We must convert back time to int to avoid rendering issues
    scatter_source = ColumnDataSource(
        data=dict(cl=flat_data[:, 0], time=flat_data[:, 1].astype(int))
    )
    # We use jitter to avoid plotting all class timing on the same X
    p.scatter(
        jitter("cl", 0.3, range=p.x_range),
        "time",
        source=scatter_source,
        alpha=0.5,
        size=13,
        line_color="white",
        color=factor_cmap("cl", "Light7", classes),
    )
    return p


def plot(datas: list[Data]):
    figures = []
    # Batch data by 7 because we use Light7 as color palette
    for data_batch in itertools.batched(datas, 7):
        figures.append(plot_batch(list(data_batch)))
    show(
        column(
            figures,
            sizing_mode="stretch_width",
        )
    )


def is_file(file: str) -> Path:
    p = Path(file)
    if not p.is_file():
        raise argparse.ArgumentTypeError(f"{file} is not a file")
    return p


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot benchmark results")
    parser.add_argument("json_output", help="Include directory", type=is_file)
    return parser


def main(args: list[str]):
    parser = argument_parser()
    args = parser.parse_args(args)

    datas = parse_json_output(json.loads(args.json_output.open().read()))
    plot(datas)


if __name__ == "__main__":
    import sys

    main(sys.argv[1:])
