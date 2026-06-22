# A single outing belonging to a race weekend or test.
# Contains driver, environment, car state, and a reference to the data file.

from sqlalchemy import Column, Integer, String, Float, DateTime, ForeignKey
from sqlalchemy.orm import relationship
from models.base import Base

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
    lap_selection = Column(String(20), nullable=True)
    session_type = Column(String(20), nullable=True)
    comments = Column(String(5000), nullable=True)
    
                          

    # relationships
    race_weekend = relationship("RaceWeekend", back_populates="outings")
    driver = relationship("Driver")