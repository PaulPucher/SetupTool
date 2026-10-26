# SQLite engine, session factory and declarative base; all models import
# Base from here.

import os

from sqlalchemy import create_engine, text
from sqlalchemy.orm import DeclarativeBase, sessionmaker

DATABASE_PATH = "data/setuptool.db"

# SQLite does not create missing parent directories (fresh checkout).
os.makedirs(os.path.dirname(DATABASE_PATH), exist_ok=True)

engine = create_engine(f"sqlite:///{DATABASE_PATH}", echo=False)  # echo=True logs SQL

Session = sessionmaker(bind=engine)

class Base(DeclarativeBase):
    pass

def _migrate_add_missing_columns():
    # create_all adds tables, not columns -> ALTER TABLE ADD COLUMN, checked
    # via PRAGMA table_info each start (idempotent)
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