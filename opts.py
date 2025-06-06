import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "-a",
    "--algo",
    choices=["cbs-gcs"],
    default="cbs-gcs",
)
parser.add_argument("-t", "--maxt", default=-1.0, type=float)
parser.add_argument("-tlimit", default=-1, type=int)
parser.add_argument("-dt", "--stepsize", default=0.2, type=float)
parser.add_argument("-v", "--maxv", type=float)
args = parser.parse_args()
