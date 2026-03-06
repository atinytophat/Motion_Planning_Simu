# Windows Setup Instructions

Due to PowerShell execution policy restrictions on your system, here are the recommended approaches to install and run this project:

## Method 1: Using Command Prompt (CMD) - RECOMMENDED

1. **Open Command Prompt:**
   - Press `Win + R`
   - Type `cmd` and press Enter

2. **Navigate to project folder:**
   ```cmd
   cd /d "C:\Users\Isamu\OneDrive\Desktop\Programming\ME5463 Real Time\RoboDK_Motion_Planning_Simu\Motion_Simu"
   ```

3. **Install dependencies:**
   ```cmd
   npm install
   ```

4. **Start development server:**
   ```cmd
   npm run dev
   ```

---

## Method 2: Using Batch File

1. **Double-click `install.bat`** in the project folder
2. Wait for installation to complete
3. Open Command Prompt and run:
   ```cmd
   npm run dev
   ```

---

## Method 3: Fix PowerShell Policy (Advanced)

If you want to use PowerShell with npm, temporarily change execution policy:

1. **Open PowerShell as Administrator:**
   - Press `Win + X`
   - Select "Windows PowerShell (Admin)"

2. **Change execution policy:**
   ```powershell
   Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
   ```

3. **Confirm the change** when prompted

4. **Now you can use npm normally:**
   ```powershell
   npm install
   npm run dev
   ```

**Note:** This change affects only your user account and is safe.

---

## Method 4: Using VS Code Terminal

If using Visual Studio Code:

1. **Open integrated terminal:** `Ctrl + ``
2. **Change terminal to CMD:**
   - Click the dropdown arrow in terminal
   - Select "Command Prompt"
3. **Run npm commands:**
   ```cmd
   npm install
   npm run dev
   ```

---

## Verifying Installation

After successful installation, you should see:

1. **`node_modules` folder** in the project directory
2. **`package-lock.json` file** created
3. Development server starts and opens in browser

---

## Troubleshooting Windows-Specific Issues

### Issue: "npm: The term 'npm' is not recognized"
- Node.js may not be installed or not in PATH
- Install from: https://nodejs.org/
- Restart Command Prompt after installation

### Issue: "Access denied" errors
- Run Command Prompt as Administrator
- Or use Method 3 to permanently fix PowerShell policy

### Issue: Port 3000 in use
- Run in CMD: `netstat -ano | findstr :3000`
- Note the PID (Process ID)
- Run: `taskkill /PID [PID] /F`
- Or change port in `vite.config.js`

### Issue: Long paths error (path > 260 characters)
- Enable long path support in Windows or move project to shorter path
- Example: `C:\Dev\motion-planning` instead of current location

---

## Quick Reference Commands

| Task | Command |
|------|---------|
| Install dependencies | `npm install` |
| Start dev server | `npm run dev` |
| Build for production | `npm run build` |
| Preview build | `npm run preview` |
| Check for errors | `npm run lint` |
| Update dependencies | `npm update` |

---

## Environment Variables on Windows

If you need to set environment variables, create `.env.local` file:

```bash
# Copy .env.example to .env.local
copy .env.example .env.local
```

Then edit `.env.local` with your values.

---

## Getting Help

1. Check the QUICKSTART.md file for usage instructions
2. Read README.md for detailed documentation
3. Check browser console (F12) for JavaScript errors
4. Ensure Node.js version is 16+: `node --version`
5. Clear npm cache if issues persist: `npm cache clean --force`

---

**Last Updated:** March 4, 2026
**System:** Windows with PowerShell policy restrictions
