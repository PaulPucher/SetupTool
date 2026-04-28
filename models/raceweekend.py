# define a raceweekend or test as folder for outings


from sqlalchemy import Column, Integer, String, ForeignKey
from sqlalchemy.orm import relationship
from models.base import Base

class RaceWeekend(Base):
    __tablename__ = "race_weekends"

    id = Column(Integer, primary_key=True, autoincrement=True)
    track = Column(String(100), nullable=False)
    series = Column(String(100), nullable=False)
    car_number = Column(Integer, nullable=False)
    year = Column(Integer, nullable=False)

    outings = relationship("Outing", back_populates="race_weekend")