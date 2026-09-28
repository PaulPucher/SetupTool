# List of Drivers with attributes(name, driving level)
from sqlalchemy import Column, Integer, String
from models.base import Base, Session

class Driver(Base):
    __tablename__ = "drivers"

    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String(100), nullable=False)
    driving_level = Column(Integer, nullable=True)


def driver_name_and_level(driver_id):
    # Plain values read inside a short session. Callers hold detached Outing
    # objects; going through the lazy Outing.driver relationship after the
    # session closed raises DetachedInstanceError.
    if driver_id is None:
        return None, None
    session = Session()
    driver = session.get(Driver, driver_id)
    name = driver.name if driver else None
    level = driver.driving_level if driver else None
    session.close()
    return name, level
