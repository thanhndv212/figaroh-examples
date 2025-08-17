# filepath: /figaroh-examples/web-interface/core/session_manager.py

class SessionManager:
    def __init__(self):
        self.sessions = {}

    def create_session(self, user_id):
        """Create a new session for a user."""
        session_id = self._generate_session_id()
        self.sessions[session_id] = {'user_id': user_id, 'data': {}}
        return session_id

    def get_session(self, session_id):
        """Retrieve a session by its ID."""
        return self.sessions.get(session_id)

    def delete_session(self, session_id):
        """Delete a session by its ID."""
        if session_id in self.sessions:
            del self.sessions[session_id]

    def _generate_session_id(self):
        """Generate a unique session ID."""
        import uuid
        return str(uuid.uuid4())
    
    def cleanup(self):
        """Clean up session manager resources."""
        try:
            print("🔧 Shutting down session manager...")
            session_count = len(self.sessions)
            if session_count > 0:
                print(f"🧹 Cleaning up {session_count} active sessions...")
                self.sessions.clear()
            print("✅ Session manager shutdown complete")
        except Exception as e:
            print(f"❌ Error during session manager cleanup: {e}")