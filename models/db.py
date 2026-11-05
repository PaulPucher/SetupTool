# SQLite storage of the app's records: race weekends, their outings and the
# drivers. Engine and session factory first, then the three tables.

import os

from sqlalchemy import Column, Date, DateTime, Float, ForeignKey, Integer, String, create_engine
from sqlalchemy.orm import DeclarativeBase, relationship, sessionmaker

from modules.storage import repo_path

DATABASE_PATH = repo_path("data", "setuptool.db")

# SQLite does not create missing parent directories (fresh checkout).
os.makedirs(os.path.dirname(DATABASE_PATH), exist_ok=True)

engine = create_engine(f"sqlite:///{DATABASE_PATH}", echo=False)  # echo=True logs SQL

Session = sessionmaker(bind=engine)


class Base(DeclarativeBase):
    pass


class Driver(Base):
    __tablename__ = "drivers"

    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String(100), nullable=False)
    driving_level = Column(Integer, nullable=True)


# Race weekend or test; parent of its outings.
class RaceWeekend(Base):
    __tablename__ = "race_weekends"

    id = Column(Integer, primary_key=True, autoincrement=True)
    track = Column(String(100), nullable=False)
    series = Column(String(100), nullable=False)
    car_number = Column(Integer, nullable=False)
    year = Column(Integer, nullable=False)
    date = Column(Date, nullable=True)
    type = Column(String(20), nullable=True)


# A single outing of a race weekend or test: driver, environment, car state,
# setup sheets and a reference to the data file.
class Outing(Base):
    __tablename__ = "outings"

    id = Column(Integer, primary_key=True, autoincrement=True)
    date_time = Column(DateTime, nullable=False)
    name = Column(String(100), nullable=True)
    number = Column(Integer, nullable=True)

    # links
    race_weekend_id = Column(Integer, ForeignKey("race_weekends.id"), nullable=False)
    driver_id = Column(Integer, ForeignKey("drivers.id"), nullable=True)

    # environment
    air_temp = Column(Float, nullable=True)
    track_temp = Column(Float, nullable=True)
    track_condition = Column(String(20), nullable=True)

    # car state
    fuel_level = Column(Float, nullable=True)
    tyre_age = Column(Integer, nullable=True)
    tyre_type = Column(String(10), nullable=True)
    tyre_name = Column(String(100), nullable=True)

    # car setup - stored as JSON string
    setup_data = Column(String(10000), nullable=True)
    setdown_data = Column(String(10000), nullable=True)
    feedback_data = Column(String(10000), nullable=True)

    # data reference
    csv_path = Column(String(500), nullable=True)
    session_type = Column(String(20), nullable=True)
    comments = Column(String(5000), nullable=True)

    # cached analysis payload as JSON (OutingForm._build_analysis_data_json),
    # no length cap. Corner verdicts are not stored; they are classified
    # live from current config.
    analysis_data = Column(String, nullable=True)

    driver = relationship("Driver")


def init_db():
    Base.metadata.create_all(engine)
