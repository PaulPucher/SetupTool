# List of Drivers with attributes(name, driving level) 
from sqlalchemy import Column, Integer, String
from models.base import Base

class Driver(Base):
    __tablename__ = "drivers"

    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String(100), nullable=False)
    driving_level = Column(Integer, nullable=True)
  