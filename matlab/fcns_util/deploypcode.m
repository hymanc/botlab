function deploypcode(sourcedir, targetdir, ignore)
%DEPLOYPCODE  Generates pcode with .m help files.
%   DEPLOYPCODE(SOURCEDIR,TARGETDIR) scans the matlab .m files in the
%   SOURCEDIR and generates pcode .p file versions in the TARGETDIR.  If
%   TARGETDIR does not exist it will be created.  Also copies .m help files
%   into the TARGETDIR.  Any files in SOURCEDIR that are unrecognized will be
%   copied over "as-is".  Does not recursively follow SOURCEDIR.
%
%   DEPLOYPCODE(SOURCEDIR,TARGETDIR,IGNORE) ignores the files specified in
%   IGNORE, where IGNORE is a cell array of files to exclude (can be regex
%   expressions). For example IGNORE={'.gitignore','.svn','foo_*.m'}.

if strcmp(sourcedir, targetdir)
    error('sourcedir [%s] and targetdir [%s] are the same, aborting', ...
          sourcedir, targetdir);
end

if isdir(targetdir)
    error('targetdir [%s] already exists, aborting', targetdir);
end

cmd = sprintf('cp -ra %s %s', sourcedir, targetdir);
[s, r] = unix(cmd);
if s
    error(r);
end

pcode(targetdir,'-INPLACE');
strip_mfiles(targetdir, ignore);

%===============================================================================
function strip_mfiles(targetdir, ignore)

d = dir(targetdir);
for f=1:length(d)
    if d(f).isdir
        if strcmp(d(f).name,'.') || strcmp(d(f).name,'..')
            continue;
        else
            strip_mfiles(fullfile(targetdir, d(f).name));
            continue;
        end
    end

    for r=1:length(ignore)
        if regexp(d(f).name, ignore{r})
            fprintf('skipping %s, matches regexp %s\n', ...
                    d(f).name, ignore{r});
            continue;
        end
    end

    [pathstr, name, ext, versn] = fileparts(d(f).name);
    if strcmpi(ext, '.m')
        mfile = fullfile(targetdir, d(f).name);
        fid = fopen(mfile);
        if fid < 0
            error('could not open file %s', mfile);
        end

        % parse help text from m-file
        thelp = {};
        started = 0;
        done = 0;
        while ~done
            tline = fgets(fid);
            s = strtrim(tline);
            if ~ischar(tline)
                break;
            end

            if ~isempty(s)
                if s(1) == '%'
                    started = 1;
                    thelp{end+1} = tline;
                else
                    if strncmp(s, 'function', 8)
                        continue;
                    elseif ~started
                        % no help in this file
                        break;
                    end
                end
            else
                if started
                    done = 1;
                end
            end
        end
        fclose(fid);

        % overwrite the m-file with just its help text
        if done
            fid = fopen(mfile, 'w+');
            for t=1:length(thelp)
                fprintf(fid, '%s', thelp{t});
            end
            fclose(fid);
        else
            delete(mfile);
        end

    end
end
