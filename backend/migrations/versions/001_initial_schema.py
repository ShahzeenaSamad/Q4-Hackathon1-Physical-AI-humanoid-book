"""initial_schema

Revision ID: 001
Revises:
Create Date: 2025-12-30 10:00:00.000000

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision = '001'
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    # --- Create chat_sessions table ---
    op.create_table(
        'chat_sessions',
        sa.Column('id', sa.UUID(), nullable=False),
        sa.Column('user_id', sa.UUID(), nullable=True),
        sa.Column('messages', postgresql.JSONB(astext_type=sa.Text()), server_default='[]', nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_chat_sessions_user_id'), 'chat_sessions', ['user_id'], unique=False)

    # --- Create textbook_content table ---
    op.create_table(
        'textbook_content',
        sa.Column('id', sa.UUID(), nullable=False),
        sa.Column('module_id', sa.Integer(), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('section_title', sa.String(length=255), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('embedding_id', sa.String(length=100), nullable=False),
        sa.Column('content_type', sa.String(length=50), nullable=True),
        sa.Column('learning_outcomes', sa.ARRAY(sa.String()), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_textbook_content_chapter_id'), 'textbook_content', ['chapter_id'], unique=False)
    op.create_index(op.f('ix_textbook_content_embedding_id'), 'textbook_content', ['embedding_id'], unique=True)
    op.create_index(op.f('ix_textbook_content_module_id'), 'textbook_content', ['module_id'], unique=False)


def downgrade() -> None:
    op.drop_index(op.f('ix_textbook_content_module_id'), table_name='textbook_content')
    op.drop_index(op.f('ix_textbook_content_embedding_id'), table_name='textbook_content')
    op.drop_index(op.f('ix_textbook_content_chapter_id'), table_name='textbook_content')
    op.drop_table('textbook_content')
    op.drop_index(op.f('ix_chat_sessions_user_id'), table_name='chat_sessions')
    op.drop_table('chat_sessions')
