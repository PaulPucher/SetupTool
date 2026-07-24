# Central database setup. All models import Base from here.
# engine connects to the SQLite file, Session is used to open database transactions.

from sqlalchemy import create_engine, text
from sqlalchemy.orm import DeclarativeBase, sessionmaker

DATABASE_PATH = "data/setuptool.db"

engine = create_engine(f"sqlite:///{DATABASE_PATH}", echo=False) #debugging echo=true

Session = sessionmaker(bind=engine)

class Base(DeclarativeBase):
    pass

def _migrate_add_missing_columns():
    # create_all only creates missing TABLES, not missing columns on
    # existing ones -- SQLite ALTER TABLE ADD COLUMN is the safe, additive
    # path (no drop/recreate). Idempotent: checked via PRAGMA table_info
    # every startup, so re-running this is always a no-op once applied.
    with engine.connect() as conn:
        cols = [row[1] for row in conn.execute(text("PRAGMA table_info(outings)"))]
        if "analysis_data" not in cols:
            conn.execute(text("ALTER TABLE outings ADD COLUMN analysis_data TEXT"))
            conn.commit()

def init_db():
    # imports trigger model registration with Base
    import models.driver
    import models.raceweekend
    import models.outing

    Base.metadata.create_all(engine)
    _migrate_add_missing_columns()