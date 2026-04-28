# Central database setup. All models import Base from here.
# engine connects to the SQLite file, Session is used to open database transactions.

from sqlalchemy import create_engine
from sqlalchemy.orm import DeclarativeBase, sessionmaker

DATABASE_PATH = "data/setuptool.db"

engine = create_engine(f"sqlite:///{DATABASE_PATH}", echo=False) #debugging echo=true

Session = sessionmaker(bind=engine)

class Base(DeclarativeBase):
    pass

def init_db():
    # imports trigger model registration with Base
    import models.driver
    import models.raceweekend
    import models.outing
    
    Base.metadata.create_all(engine)