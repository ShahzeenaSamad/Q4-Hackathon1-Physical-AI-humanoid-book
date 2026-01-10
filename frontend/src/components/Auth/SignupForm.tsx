import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const SignupForm: React.FC<{ onSuccess: () => void }> = ({ onSuccess }) => {
  const { login } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  // @ts-ignore
  const backendUrl = siteConfig.themeConfig.chatWidget?.backendUrl || 'http://localhost:8000';

  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    software_background: 'beginner',
    hardware_background: 'none',
    preferred_complexity: 'standard'
  });
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsLoading(true);

    try {
      const response = await fetch(`${backendUrl}/api/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(formData),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Signup failed');
      }

      const data = await response.json();
      login(data.access_token, { id: data.user_id, name: data.name });
      onSuccess();
    } catch (err: any) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div style={{ maxWidth: '400px', margin: '2rem auto', padding: '1rem', border: '1px solid var(--ifm-toc-border-color)', borderRadius: '8px' }}>
      <h2>Sign Up</h2>
      {error && <div style={{ color: 'red', marginBottom: '1rem' }}>{error}</div>}
      <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
        <div>
          <label>Name</label>
          <input type="text" required value={formData.name} onChange={e => setFormData({ ...formData, name: e.target.value })} style={{ width: '100%', padding: '0.5rem' }} />
        </div>
        <div>
          <label>Email</label>
          <input type="email" required value={formData.email} onChange={e => setFormData({ ...formData, email: e.target.value })} style={{ width: '100%', padding: '0.5rem' }} />
        </div>
        <div>
          <label>Password (min 8 chars)</label>
          <input type="password" required minLength={8} value={formData.password} onChange={e => setFormData({ ...formData, password: e.target.value })} style={{ width: '100%', padding: '0.5rem' }} />
        </div>

        <hr />
        <h4>Background Questions</h4>

        <div>
          <label>Software Background</label>
          <select value={formData.software_background} onChange={e => setFormData({ ...formData, software_background: e.target.value })} style={{ width: '100%', padding: '0.5rem' }}>
            <option value="beginner">Beginner (No coding experience)</option>
            <option value="intermediate">Intermediate (Know Python/C++)</option>
            <option value="advanced">Advanced (Experienced Roboticist)</option>
          </select>
        </div>

        <div>
          <label>Hardware Background</label>
          <select value={formData.hardware_background} onChange={e => setFormData({ ...formData, hardware_background: e.target.value })} style={{ width: '100%', padding: '0.5rem' }}>
            <option value="none">None (Never used a robot)</option>
            <option value="hobbyist">Hobbyist (Arduino/Raspi/Drones)</option>
            <option value="professional">Professional (Industrial/Research)</option>
          </select>
        </div>

        <div>
          <label>Learning Preference</label>
          <select value={formData.preferred_complexity} onChange={e => setFormData({ ...formData, preferred_complexity: e.target.value })} style={{ width: '100%', padding: '0.5rem' }}>
            <option value="simplified">Simple (High-level concepts first)</option>
            <option value="standard">Standard (Balanced)</option>
            <option value="detailed">Detailed (Deep technical deep dives)</option>
          </select>
        </div>

        <button type="submit" disabled={isLoading} style={{ padding: '0.75rem', backgroundColor: 'var(--ifm-color-primary)', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}>
          {isLoading ? 'Creating Account...' : 'Create Account'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;
